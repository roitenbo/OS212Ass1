#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

uint64 placeIndexCounter = 0;

struct spinlock indexCounterLock;

struct cpu cpus[NCPU];

struct proc proc[NPROC];

struct proc *initproc;

int nextpid = 1;
struct spinlock pid_lock;

extern void forkret(void);
static void freeproc(struct proc *p);

extern char trampoline[]; // trampoline.S

// helps ensure that wakeups of wait()ing
// parents are not lost. helps obey the
// memory model when using p->parent.
// must be acquired before any p->lock.
struct spinlock wait_lock;

// Allocate a page for each process's kernel stack.
// Map it high in memory, followed by an invalid
// guard page.
void
proc_mapstacks(pagetable_t kpgtbl) {
  struct proc *p;
  
  for(p = proc; p < &proc[NPROC]; p++) {
    char *pa = kalloc();
    if(pa == 0)
      panic("kalloc");
    uint64 va = KSTACK((int) (p - proc));
    kvmmap(kpgtbl, va, (uint64)pa, PGSIZE, PTE_R | PTE_W);
  }
}

// initialize the proc table at boot time.
void
procinit(void)
{
  struct proc *p;
  initlock(&indexCounterLock, "placeIndexCounter");
  initlock(&pid_lock, "nextpid");
  initlock(&wait_lock, "wait_lock");
  for(p = proc; p < &proc[NPROC]; p++) {
      initlock(&p->lock, "proc");
      p->kstack = KSTACK((int) (p - proc));
  }
}

// Must be called with interrupts disabled,
// to prevent race with process being moved
// to a different CPU.
int
cpuid()
{
  int id = r_tp();
  return id;
}

// Return this CPU's cpu struct.
// Interrupts must be disabled.
struct cpu*
mycpu(void) {
  int id = cpuid();
  struct cpu *c = &cpus[id];
  return c;
}

// Return the current struct proc *, or zero if none.
struct proc*
myproc(void) {
  push_off();
  struct cpu *c = mycpu();
  struct proc *p = c->proc;
  pop_off();
  return p;
}

int
allocpid() {
  int pid;
  
  acquire(&pid_lock);
  pid = nextpid;
  nextpid = nextpid + 1;
  release(&pid_lock);

  return pid;
}

// Look in the process table for an UNUSED proc.
// If found, initialize state required to run in the kernel,
// and return with p->lock held.
// If there are no free procs, or a memory allocation fails, return 0.
static struct proc*
allocproc(void)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    acquire(&p->lock);
    if(p->state == UNUSED) {
      goto found;
    } else {
      release(&p->lock);
    }
  }
  return 0;

found:
  p->pid = allocpid();
  p->state = USED;
  p->mask = 0;
  // Allocate a trapframe page.
  if((p->trapframe = (struct trapframe *)kalloc()) == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  // An empty user page table.
  p->pagetable = proc_pagetable(p);
  if(p->pagetable == 0){
    freeproc(p);
    release(&p->lock);
    return 0;
  }

  //task3 update creation time
  acquire(&tickslock);
  p->ctime = ticks;
  //task4
  acquire(&indexCounterLock);
  p->arrayPlace = placeIndexCounter;
  placeIndexCounter += 1;
  release(&indexCounterLock);
  //task3+4 avg_bursttime
  p->average_bursttime = QUANTUM*100; // A1 = QUANTUM*100 , A(i+1)=αBi+((100-α)Ai)/100
  p->startBurstTime = ticks;
  //priority normal
  p->myPriority = 5; // normal priority 3 which is 5
  release(&tickslock);
  // as in sysproc.c


  // Set up new context to start executing at forkret,
  // which returns to user space.
  memset(&p->context, 0, sizeof(p->context));
  p->context.ra = (uint64)forkret;
  p->context.sp = p->kstack + PGSIZE;

  return p;
}

// free a proc structure and the data hanging from it,
// including user pages.
// p->lock must be held.
static void
freeproc(struct proc *p)
{
  if(p->trapframe)
    kfree((void*)p->trapframe);
  p->trapframe = 0;
  if(p->pagetable)
    proc_freepagetable(p->pagetable, p->sz);
  p->pagetable = 0;
  p->sz = 0;
  p->pid = 0;
  p->parent = 0;
  p->name[0] = 0;
  p->chan = 0;
  p->killed = 0;
  p->xstate = 0;
  p->state = UNUSED;
  //task3
  p->ctime = 0;
  p->ttime = 0;
  p->stime = 0;
  p->retime = 0;
  p->rutime = 0;
  p->average_bursttime = 0;
  p->startBurstTime = 0;
  //task4
  p->arrayPlace = -1;
  p->myPriority = 0;
  p->ticks = 0;
}

// Create a user page table for a given process,
// with no user memory, but with trampoline pages.
pagetable_t
proc_pagetable(struct proc *p)
{
  pagetable_t pagetable;

  // An empty page table.
  pagetable = uvmcreate();
  if(pagetable == 0)
    return 0;

  // map the trampoline code (for system call return)
  // at the highest user virtual address.
  // only the supervisor uses it, on the way
  // to/from user space, so not PTE_U.
  if(mappages(pagetable, TRAMPOLINE, PGSIZE,
              (uint64)trampoline, PTE_R | PTE_X) < 0){
    uvmfree(pagetable, 0);
    return 0;
  }

  // map the trapframe just below TRAMPOLINE, for trampoline.S.
  if(mappages(pagetable, TRAPFRAME, PGSIZE,
              (uint64)(p->trapframe), PTE_R | PTE_W) < 0){
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}

// Free a process's page table, and free the
// physical memory it refers to.
void
proc_freepagetable(pagetable_t pagetable, uint64 sz)
{
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmfree(pagetable, sz);
}

// a user program that calls exec("/init")
// od -t xC initcode
uchar initcode[] = {
  0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x45, 0x02,
  0x97, 0x05, 0x00, 0x00, 0x93, 0x85, 0x35, 0x02,
  0x93, 0x08, 0x70, 0x00, 0x73, 0x00, 0x00, 0x00,
  0x93, 0x08, 0x20, 0x00, 0x73, 0x00, 0x00, 0x00,
  0xef, 0xf0, 0x9f, 0xff, 0x2f, 0x69, 0x6e, 0x69,
  0x74, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

// Set up first user process.
void
userinit(void)
{
  struct proc *p;

  p = allocproc();
  initproc = p;
  
  // allocate one user page and copy init's instructions
  // and data into it.
  uvminit(p->pagetable, initcode, sizeof(initcode));
  p->sz = PGSIZE;

  // prepare for the very first "return" from kernel to user.
  p->trapframe->epc = 0;      // user program counter
  p->trapframe->sp = PGSIZE;  // user stack pointer

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  p->state = RUNNABLE;

  release(&p->lock);
}

// Grow or shrink user memory by n bytes.
// Return 0 on success, -1 on failure.
int
growproc(int n)
{
  uint sz;
  struct proc *p = myproc();

  sz = p->sz;
  if(n > 0){
    if((sz = uvmalloc(p->pagetable, sz, sz + n)) == 0) {
      return -1;
    }
  } else if(n < 0){
    sz = uvmdealloc(p->pagetable, sz, sz + n);
  }
  p->sz = sz;
  return 0;
}

// Create a new process, copying the parent.
// Sets up child kernel stack to return as if from fork() system call.
int
fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *p = myproc();

  // Allocate process.
  if((np = allocproc()) == 0){
    return -1;
  }

  // Copy user memory from parent to child.
  if(uvmcopy(p->pagetable, np->pagetable, p->sz) < 0){
    freeproc(np);
    release(&np->lock);
    return -1;
  }
  np->sz = p->sz;

  //task2
  np->mask = p->mask; // copy mask to np
  //task2

  //task3
  p->average_bursttime = QUANTUM*100; // A1 = QUANTUM*100 , A(i+1)=αBi+((100-α)Ai)/100
  //task3

  //task4
  np->myPriority = p->myPriority; // child has same priority as his parent
  //task4

  // copy saved user registers.
  *(np->trapframe) = *(p->trapframe);

  // Cause fork to return 0 in the child.
  np->trapframe->a0 = 0;

  // increment reference counts on open file descriptors.
  for(i = 0; i < NOFILE; i++)
    if(p->ofile[i])
      np->ofile[i] = filedup(p->ofile[i]);
  np->cwd = idup(p->cwd);

  safestrcpy(np->name, p->name, sizeof(p->name));

  pid = np->pid;

  release(&np->lock);

  acquire(&wait_lock);
  np->parent = p;
  release(&wait_lock);

  acquire(&np->lock);
  np->state = RUNNABLE;
  acquire(&indexCounterLock);
  np->arrayPlace = placeIndexCounter;
  placeIndexCounter += 1;
  release(&indexCounterLock);
  release(&np->lock);

  return pid;
}

// Pass p's abandoned children to init.
// Caller must hold wait_lock.
void
reparent(struct proc *p)
{
  struct proc *pp;

  for(pp = proc; pp < &proc[NPROC]; pp++){
    if(pp->parent == p){
      pp->parent = initproc;
      wakeup(initproc);
    }
  }
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait().
void
exit(int status)
{
  struct proc *p = myproc();

  if(p == initproc)
    panic("init exiting");

  // Close all open files.
  for(int fd = 0; fd < NOFILE; fd++){
    if(p->ofile[fd]){
      struct file *f = p->ofile[fd];
      fileclose(f);
      p->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(p->cwd);
  end_op();
  p->cwd = 0;

  acquire(&wait_lock);

  // Give any children to init.
  reparent(p);

  // Parent might be sleeping in wait().
  wakeup(p->parent);
  
  acquire(&p->lock);

  p->xstate = status;
  p->state = ZOMBIE;

  //task3 update termination time
  acquire(&tickslock);
  p->ttime = ticks;
  release(&tickslock);
  // as in sysproc.c

  release(&wait_lock);

  // Jump into the scheduler, never to return.
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int
wait(uint64 addr)
{
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();

  acquire(&wait_lock);

  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(np = proc; np < &proc[NPROC]; np++){
      if(np->parent == p){
        // make sure the child isn't still in exit() or swtch().
        acquire(&np->lock);

        havekids = 1;
        if(np->state == ZOMBIE){
          // Found one.
          pid = np->pid;
          if(addr != 0 && copyout(p->pagetable, addr, (char *)&np->xstate,
                                  sizeof(np->xstate)) < 0) {
            release(&np->lock);
            release(&wait_lock);
            return -1;
          }
          freeproc(np);
          release(&np->lock);
          release(&wait_lock);
          return pid;
        }
        release(&np->lock);
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || p->killed){
      release(&wait_lock);
      return -1;
    }
    
    // Wait for a child to exit.
    sleep(p, &wait_lock);  //DOC: wait-sleep
  }
}

// Per-CPU process scheduler.
// Each CPU calls scheduler() after setting itself up.
// Scheduler never returns.  It loops, doing:
//  - choose a process to run.
//  - swtch to start running that process.
//  - eventually that process transfers control
//    via swtch back to the scheduler.
#ifdef DEFAULT
void
scheduler(void)
{
  //printf("DEFAULT%s\n","");
  struct proc *p;
  struct cpu *c = mycpu();
  
  c->proc = 0;
  for(;;){
    // Avoid deadlock by ensuring that devices can interrupt.
    intr_on();

    for(p = proc; p < &proc[NPROC]; p++) {
      acquire(&p->lock);
      if(p->state == RUNNABLE) {
        // Switch to chosen process.  It is the process's job
        // to release its lock and then reacquire it
        // before jumping back to us.
        p->state = RUNNING;
        c->proc = p;
        //task4
        p->ticks = 0; //reseting the ticks of the new process who gets cpu
        //task4
        //task3+4 start of BurstTime 
        acquire(&tickslock);
        p->startBurstTime = ticks;
        release(&tickslock);
        //task3+4 start of BurstTime 

        swtch(&c->context, &p->context);

        // Process is done running for now.
        // It should have changed its p->state before coming back.
        c->proc = 0;
      }
      release(&p->lock);
    }
  }
}
#endif


#ifdef FCFS
void
scheduler(void)
{
  //printf("FCFS%s\n","");
  struct proc *minP = 0;
  struct proc *p;
  struct cpu *c = mycpu();
  
  c->proc = 0;
  for(;;){
    intr_on();
    minP = 0;
    for(p = proc; p < &proc[NPROC]; p++) {
      acquire(&p->lock);
      if(p->state == RUNNABLE) {
        if(minP == 0 && p->arrayPlace != -1){
          minP = p;
        }        
        else{
          if(minP->arrayPlace > p->arrayPlace && p->arrayPlace != -1){
            minP = p;
          }
        }
      }
      release(&p->lock);
    }
    if(minP != 0 && minP->state == RUNNABLE){
        //printf("minp = %d\n",minP->arrayPlace);
        acquire(&minP->lock);
        minP->state = RUNNING;
        c->proc = minP;
        //task4
        minP->ticks = 0; //reseting the ticks of the new process who gets cpu
        //task4

        //task3+4 start of BurstTime 
        acquire(&tickslock);
        p->startBurstTime = ticks;
        release(&tickslock);
        //task3+4 start of BurstTime 

        swtch(&c->context, &minP->context);

        if(minP->state != RUNNABLE)
          minP->arrayPlace = -1;

        // Process is done running for now.
        // It should have changed its p->state before coming back.
        c->proc = 0;
        release(&minP->lock);
    }
  }
}
#endif

#ifdef SRT
void
scheduler(void)
{
  //printf("SRT%s\n","");
  struct proc *minP = 0;
  struct proc *p;
  struct cpu *c = mycpu();
  
  c->proc = 0;
  for(;;){
    intr_on();
    minP = 0;
    for(p = proc; p < &proc[NPROC]; p++) {
      acquire(&p->lock);
      if(p->state == RUNNABLE) {
        if(minP == 0){
          minP = p;
        }        
        else{
          if(minP->average_bursttime > p->average_bursttime){
            minP = p;
          }
        }
      }
      release(&p->lock);
    }
    if(minP != 0 && minP->state == RUNNABLE){
        //printf("minp = %d\n",minP->pid);
        acquire(&minP->lock);
        minP->state = RUNNING;
        c->proc = minP;
        //task4
        minP->ticks = 0; //reseting the ticks of the new process who gets cpu
        //task4

        //task3+4 start of BurstTime 
        acquire(&tickslock);
        p->startBurstTime = ticks;
        release(&tickslock);
        //task3+4 start of BurstTime 

        swtch(&c->context, &minP->context);

        // Process is done running for now.
        // It should have changed its p->state before coming back.
        c->proc = 0;
        release(&minP->lock);
    }
  }
}
#endif


#ifdef CFSD
int cfsdFormula(struct proc *p){ // already locked
  if((p->rutime + p->stime) != 0) // cant divide by zero
    return (p->rutime) * (p->myPriority) / (p->rutime + p->stime);
  return 0;
}

void
scheduler(void)
{
  //printf("CFSD%s\n","");
   struct proc *minP = 0;
  struct proc *p;
  struct cpu *c = mycpu();
  
  c->proc = 0;
  for(;;){
    intr_on();
    minP = 0;
    for(p = proc; p < &proc[NPROC]; p++) {
      acquire(&p->lock);
      if(p->state == RUNNABLE) {
        if(minP == 0){
          minP = p;
        }        
        else{
          if(cfsdFormula(minP) > cfsdFormula(p)){
            minP = p;
          }
        }
      }
      release(&p->lock);
    }
    if(minP != 0 && minP->state == RUNNABLE){
        //printf("minp = %d\n",minP->pid);
        acquire(&minP->lock);
        minP->state = RUNNING;
        c->proc = minP;
        //task4
        minP->ticks = 0; //reseting the ticks of the new process who gets cpu
        //task4

        //task3+4 start of BurstTime 
        acquire(&tickslock);
        p->startBurstTime = ticks;
        release(&tickslock);
        //task3+4 start of BurstTime 

        swtch(&c->context, &minP->context);

        // Process is done running for now.
        // It should have changed its p->state before coming back.
        c->proc = 0;
        release(&minP->lock);
    }
  }
}
#endif

// Switch to scheduler.  Must hold only p->lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->noff, but that would
// break in the few places where a lock is held but
// there's no process.
void
sched(void)
{
  int intena;
  struct proc *p = myproc();
  
  //task3+4 avgBurst
  
  int currentBurst;
  //task4 fcfs proccess yield -> back of the queue
  acquire(&tickslock);
  currentBurst = ticks - p->startBurstTime; // calculation of bursttime
  release(&tickslock);
  //task4 fcfs
  p->average_bursttime = ALPHA*currentBurst + (100-ALPHA)*(p->average_bursttime)/100;
  //task3+4 avgBurst
  
  if(!holding(&p->lock))
    panic("sched p->lock");
  if(mycpu()->noff != 1)
    panic("sched locks");
  if(p->state == RUNNING)
    panic("sched running");
  if(intr_get())
    panic("sched interruptible");

  intena = mycpu()->intena;
  swtch(&p->context, &mycpu()->context);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void
yield(void)
{
  struct proc *p = myproc();
  acquire(&p->lock);
  p->state = RUNNABLE;
  acquire(&indexCounterLock);
  p->arrayPlace = placeIndexCounter;
  placeIndexCounter += 1;
  release(&indexCounterLock);
  sched();
  release(&p->lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch to forkret.
void
forkret(void)
{
  static int first = 1;

  // Still holding p->lock from scheduler.
  release(&myproc()->lock);

  if (first) {
    // File system initialization must be run in the context of a
    // regular process (e.g., because it calls sleep), and thus cannot
    // be run from main().
    first = 0;
    fsinit(ROOTDEV);
  }

  usertrapret();
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void
sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  
  // Must acquire p->lock in order to
  // change p->state and then call sched.
  // Once we hold p->lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup locks p->lock),
  // so it's okay to release lk.

  acquire(&p->lock);  //DOC: sleeplock1
  release(lk);

  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;

  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  release(&p->lock);
  acquire(lk);
}

// Wake up all processes sleeping on chan.
// Must be called without any p->lock.
void
wakeup(void *chan)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++) {
    if(p != myproc()){
      acquire(&p->lock);
      if(p->state == SLEEPING && p->chan == chan) {
        p->state = RUNNABLE;
        acquire(&indexCounterLock);
        p->arrayPlace = placeIndexCounter;
        placeIndexCounter += 1;
        release(&indexCounterLock);
      }
      release(&p->lock);
    }
  }
}

// Kill the process with the given pid.
// The victim won't exit until it tries to return
// to user space (see usertrap() in trap.c).
int
kill(int pid)
{
  struct proc *p;

  for(p = proc; p < &proc[NPROC]; p++){
    acquire(&p->lock);
    if(p->pid == pid){
      p->killed = 1;
      if(p->state == SLEEPING){
        // Wake process from sleep().
        p->state = RUNNABLE;
        acquire(&indexCounterLock);
        p->arrayPlace = placeIndexCounter;
        placeIndexCounter += 1;
        release(&indexCounterLock);
      }
      release(&p->lock);
      return 0;
    }
    release(&p->lock);
  }
  return -1;
}

// Copy to either a user address, or kernel address,
// depending on usr_dst.
// Returns 0 on success, -1 on error.
int
either_copyout(int user_dst, uint64 dst, void *src, uint64 len)
{
  struct proc *p = myproc();
  if(user_dst){
    return copyout(p->pagetable, dst, src, len);
  } else {
    memmove((char *)dst, src, len);
    return 0;
  }
}

// Copy from either a user address, or kernel address,
// depending on usr_src.
// Returns 0 on success, -1 on error.
int
either_copyin(void *dst, int user_src, uint64 src, uint64 len)
{
  struct proc *p = myproc();
  if(user_src){
    return copyin(p->pagetable, dst, src, len);
  } else {
    memmove(dst, (char*)src, len);
    return 0;
  }
}

// Print a process listing to console.  For debugging.
// Runs when user types ^P on console.
// No lock to avoid wedging a stuck machine further.
void
procdump(void)
{
  static char *states[] = {
  [UNUSED]    "unused",
  [SLEEPING]  "sleep ",
  [RUNNABLE]  "runble",
  [RUNNING]   "run   ",
  [ZOMBIE]    "zombie"
  };
  struct proc *p;
  char *state;

  printf("\n");
  for(p = proc; p < &proc[NPROC]; p++){
    if(p->state == UNUSED)
      continue;
    if(p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    printf("%d %s %s", p->pid, state, p->name);
    printf("\n");
  }
}


//task2
int tracePid(int mask,int pid){
  struct proc *p;
  for(p = proc; p < &proc[NPROC]; p++){
    acquire(&p->lock);
    if(p->pid == pid){
      p->mask = mask;
      release(&p->lock);
      return 0;
    }
    release(&p->lock);
  }
  return -1;
}

//task3
int wait_stat(int* status, struct perf * performance){
  struct proc *np;
  int havekids, pid;
  struct proc *p = myproc();

  acquire(&wait_lock);

  for(;;){
    // Scan through table looking for exited children.
    havekids = 0;
    for(np = proc; np < &proc[NPROC]; np++){
      if(np->parent == p){
        // make sure the child isn't still in exit() or swtch().
        acquire(&np->lock);

        havekids = 1;
        if(np->state == ZOMBIE){
          //addition to wait system call task3 "The parent will get back its' childrens performance."
          //printf("pid kid = %d \n",np->pid);
          //printf("stime kid = %d \n",np->stime);
          if((performance != 0 && copyout(p->pagetable, (uint64)performance, (char *)&np->ctime,
                                  sizeof(struct perf)) < 0)
                                  ||
            (status != 0 && copyout(p->pagetable, (uint64)status, (char *)&np->xstate,
            sizeof(np->xstate)) < 0)
                                  ) {

            release(&np->lock);
            release(&wait_lock);
            return -1;
          }

          //addition to wait system call task3

          // Found one.
          pid = np->pid;
          
          freeproc(np);
          release(&np->lock);
          release(&wait_lock);
          return pid;
        }
        release(&np->lock);
      }
    }

    // No point waiting if we don't have any children.
    if(!havekids || p->killed){
      release(&wait_lock);
      return -1;
    }
    
    // Wait for a child to exit.
    sleep(p, &wait_lock);  //DOC: wait-sleep
  }
}

//task3 update rutime,stime,retime,average
void updateEveryTick(void){
  struct proc *p;
  for(p = proc; p < &proc[NPROC]; p++) { //ctime,ttime,stime,retime,rutime,average_bursttime
    acquire(&p->lock);
    if(p->state == RUNNING) {
      p->rutime += 1;
      //printf("\nPID %d , runtime = %d\n",p->pid,p->rutime);
    } 
    if(p->state == SLEEPING) {
      p->stime += 1;
      //printf("\nPID %d , sleeptime = %d\n",p->pid,p->stime);
    } 
    if(p->state == RUNNABLE) {
      p->retime += 1;
      //printf("\nPID %d , runnabletime = %d\n",p->pid,p->retime);
    } 
    //bursttime?
    release(&p->lock);
  }
}

//task4 count ticks to quantom
int ticksToQuantom(void){
  struct proc *p = myproc();
  acquire(&p->lock);
  p->ticks = p->ticks + 1;
  int pTicks = p->ticks;
  //printf("ticks = %d\n",pTicks);
  release(&p->lock);
  return pTicks;
}

//task4 priority
int set_priority(int priority){
  struct proc *p = myproc();
  if(priority>5 || priority<1)
    return -1;
  acquire(&p->lock);
  if(priority == 1)
    p->myPriority = 1;
  if(priority == 2)
    p->myPriority = 3;
  if(priority == 3)
    p->myPriority = 5;
  if(priority == 4)
    p->myPriority = 7;
  if(priority == 5)
    p->myPriority = 25;
  release(&p->lock);
  //printf("new priority = %d\n",p->myPriority);
  return 0;
}