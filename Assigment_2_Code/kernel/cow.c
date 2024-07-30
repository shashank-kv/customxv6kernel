#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "elf.h"
#include <stdbool.h>

struct spinlock cow_lock;

// Max number of pages a CoW group of processes can share
#define SHMEM_MAX 100

struct cow_group {
    int group; // group id
    uint64 shmem[SHMEM_MAX]; // list of pages a CoW group share
    int count; // Number of active processes
};

struct cow_group cow_group[NPROC];

struct cow_group* get_cow_group(int group) {
    if(group == -1)
        return 0;

    for(int i = 0; i < NPROC; i++) {
        if(cow_group[i].group == group)
            return &cow_group[i];
    }
    return 0;
}

void cow_group_init(int groupno) {
    for(int i = 0; i < NPROC; i++) {
        if(cow_group[i].group == -1) {
            cow_group[i].group = groupno;
            return;
        }
    }
} 

int get_cow_group_count(int group) {
    return get_cow_group(group)->count;
}
void incr_cow_group_count(int group) {
    get_cow_group(group)->count = get_cow_group_count(group)+1;
}
void decr_cow_group_count(int group) {
    get_cow_group(group)->count = get_cow_group_count(group)-1;
}

void add_shmem(int group, uint64 pa) {
    if(group == -1)
        return;

    uint64 *shmem = get_cow_group(group)->shmem;
    int index;
    for(index = 0; index < SHMEM_MAX; index++) {
        // duplicate address
        if(shmem[index] == pa)
            return;
        if(shmem[index] == 0)
            break;
    }
    shmem[index] = pa;
}

int is_shmem(int group, uint64 pa) {
    if(group == -1)
        return 0;

    uint64 *shmem = get_cow_group(group)->shmem;
    for(int i = 0; i < SHMEM_MAX; i++) {
        if(shmem[i] == 0)
            return 0;
        if(shmem[i] == pa)
            return 1;
    }
    return 0;
}

void cow_init() {
    for(int i = 0; i < NPROC; i++) {
        cow_group[i].count = 0;
        cow_group[i].group = -1;
        for(int j = 0; j < SHMEM_MAX; j++)
            cow_group[i].shmem[j] = 0;
    }
    initlock(&cow_lock, "cow_lock");
}

int uvmcopy_cow(pagetable_t old, pagetable_t new, uint64 sz) {
    
    /* CSE 536: (2.6.1) Handling Copy-on-write fork() */

    // Copy user vitual memory from old(parent) to new(child) process
    pte_t *pte;
    uint64 pa, i;
    uint flags;
    int group = myproc()->cow_group;

    for(i = 0; i < sz; i += PGSIZE){
        if((pte = walk(old, i, 0)) == 0)
        panic("uvmcopy_cow: pte should exist");
        if((*pte & PTE_V) == 0)
        panic("uvmcopy_cow: page not present");
        
        pa = PTE2PA(*pte);
        if(!is_shmem(group, pa)) {
          add_shmem(group, pa);
        }
        
        *pte &= ~PTE_W; 
        *pte |= PTE_R;
        flags = PTE_FLAGS(*pte);

        if(mappages(new, i, PGSIZE, pa, flags) != 0){
            goto err;
        }
    }
    return 0;

    // Map pages as Read-Only in both the processes

    err:
        uvmunmap(new, 0, i / PGSIZE, 1);
        return -1;
}

void copy_on_write(struct proc *p, uint64 vaddr) {
    /* CSE 536: (2.6.2) Handling Copy-on-write */
    print_copy_on_write(p, vaddr);
    // Allocate a new page 
    char *mem = kalloc();
    
    pte_t* pte = walk(p->pagetable, vaddr, 0);
    uint64 pa = PTE2PA(*pte);
    // Copy contents from the shared page to the new page
    memmove(mem, (char*)pa, PGSIZE);

    // Map the new page in the faulting process's page table with write permissions
    uint flags = PTE_FLAGS(*pte);
    flags |= PTE_W; 

    //*pte = 0;
    uvmunmap(p->pagetable, vaddr, 1, 0);
    mappages(p->pagetable, vaddr, PGSIZE, mem, flags);
}

void proc_freepagetable_cow(pagetable_t pagetable, uint64 sz, int cow_group) {
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmunmap(pagetable, TRAPFRAME, 1, 0);
    uvmfree_cow(pagetable, sz, cow_group);
    decr_cow_group_count(cow_group);
}

void
uvmunmap_cow(pagetable_t pagetable, uint64 va, uint64 npages, int do_free, int group)
{
  uint64 a;
  pte_t *pte;

  if((va % PGSIZE) != 0)
    panic("uvmunmap: not aligned");

  for(a = va; a < va + npages*PGSIZE; a += PGSIZE){
    if((pte = walk(pagetable, a, 0)) == 0)
      panic("uvmunmap: walk");
    if((*pte & PTE_V) == 0)
      continue;
      /* CSE 536: removed for on-demand allocation. */
      // panic("uvmunmap: not mapped");
    if(PTE_FLAGS(*pte) == PTE_V)
      panic("uvmunmap: not a leaf");
    if(do_free){
      uint64 pa = PTE2PA(*pte);
      
      /* CSE 536: (2.6.1) Freeing Process Memory */
      // Make sure that the shared pages, belonging to a CoW group, are not freed twice
       if(is_shmem(group, pa)){
         if(get_cow_group_count(group) == 0) kfree((void*)pa);
       } else 
        kfree((void*)pa);
    }
    *pte = 0;
  }
}

void
uvmfree_cow(pagetable_t pagetable, uint64 sz, int cow_group)
{
  if(sz > 0)
    uvmunmap_cow(pagetable, 0, PGROUNDUP(sz)/PGSIZE, 1, cow_group);
  freewalk(pagetable);
}
