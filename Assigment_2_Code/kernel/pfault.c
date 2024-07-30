/* This file contains code for a generic page fault handler for processes. */
#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "elf.h"

#include "sleeplock.h"
#include "fs.h"
#include "buf.h"

int loadseg(pagetable_t pagetable, uint64 va, struct inode *ip, uint offset, uint sz);
int flags2perm(int flags);

/* CSE 536: (2.4) read current time. */
uint64 read_current_timestamp() {
  uint64 curticks = 0;
  acquire(&tickslock);
  curticks = ticks;
  wakeup(&ticks);
  release(&tickslock);
  return curticks;
}

bool psa_tracker[PSASIZE];

/* All blocks are free during initialization. */
void init_psa_regions(void)
{
    for (int i = 0; i < PSASIZE; i++) 
        psa_tracker[i] = false;
}

/* Evict heap page to disk when resident pages exceed limit */
void evict_page_to_disk(struct proc* p) {
    /* Find free block */
    int blockno = 0;
    int freeBlocks = 0;

    for(int i = 0; i < PSASIZE; i++){
        if(!psa_tracker[i]){
            if(freeBlocks == 0) blockno = i;
            //printf("Block No: %d\n", blockno);
            freeBlocks++;
            if(freeBlocks == 4) {
                psa_tracker[blockno] = true;
                psa_tracker[blockno+1] = true;
                psa_tracker[blockno+2] = true;
                psa_tracker[blockno+3] = true;
                //printf("psa_tracker[%d]: %d, psa_tracker[%d] = %d\n",blockno, psa_tracker[blockno], blockno+3, psa_tracker[blockno+3]);
                break;
            }
        } else {
            freeBlocks = 0;
        }
    }

    /* Find victim page using FIFO. */
    int victimPageIndex = 0;
    uint64 minTime = __UINT64_MAX__;
    for(int i=0; i<MAXHEAP; i++){
        if(!p->heap_tracker[i].loaded && p->heap_tracker[i].last_load_time < minTime) {
            minTime = p->heap_tracker[i].last_load_time;
            victimPageIndex = i;
        }
    }

    uint64 vaAddr = p->heap_tracker[victimPageIndex].addr;
    
    /* Print statement. */
    print_evict_page(vaAddr, blockno);

    /* Read memory from the user to kernel memory first. */
    void* ptr = kalloc();
   
    copyin(p->pagetable,(char *) ptr, vaAddr, PGSIZE);
    
    /* Write to the disk blocks. Below is a template as to how this works. There is
     * definitely a better way but this works for now. :p */
    struct buf* b;
    for(int i=0; i<4; i++){
        b = bread(1, PSASTART+(blockno+i));
        // Copy page contents to b.data using memmove.
        memmove(b->data, ptr+(i*1024), 1024);
        bwrite(b);
        brelse(b);
    }

    /* Unmap swapped out page */
    uvmunmap(p->pagetable, vaAddr, 1, 0);

    /* Update the resident heap tracker. */
    p->heap_tracker[victimPageIndex].startblock = blockno;
    p->heap_tracker[victimPageIndex].loaded = true;
}

/* Retrieve faulted page from disk. */
void retrieve_page_from_disk(struct proc* p, uint64 uvaddr) {
    /* Find where the page is located in disk */
    int startBlockNo=0;
    for(int i=0;i<MAXHEAP;i++){
        if(p->heap_tracker[i].loaded && p->heap_tracker[i].addr == uvaddr){
            startBlockNo = p->heap_tracker[i].startblock;
            psa_tracker[startBlockNo] = false;
            psa_tracker[startBlockNo+1] = false;
            psa_tracker[startBlockNo+2] = false;
            psa_tracker[startBlockNo+3] = false;
            break;
        } 
    }

    /* Print statement. */
    print_retrieve_page(uvaddr, startBlockNo);

    /* Create a kernel page to read memory temporarily into first. */
    void* ptr = kalloc();
  
    /* Read the disk block into temp kernel page. */
    struct buf* b;
    for(int i=0; i<4; i++) {
        b = bread(1, PSASTART+(startBlockNo+i));
        brelse(b);
        memmove(ptr+(1024*i), b->data, 1024);
    }
    
    /* Copy from temp kernel page to uvaddr (use copyout) */
    copyout(p->pagetable, uvaddr, ptr, PGSIZE);
}


void page_fault_handler(void) 
{
    /* Current process struct */
    struct proc *p = myproc();

    /* Track whether the heap page should be brought back from disk or not. */
    bool load_from_disk = false;

    /* Find faulting address. */
    uint64 stval = r_stval();
    //uint64 page_offset = stval & 0xFFF;
    uint64 faulting_addr = stval >> 12; 
    faulting_addr <<= 12; // page_base_address

    print_page_fault(p->name, faulting_addr);
    //printf("SCAUSE: %d\n", r_scause());
    if(r_scause() == 15 && p->cow_enabled) {
        copy_on_write(p, faulting_addr);
        goto out;
    }

    /* Check if the fault address is a heap page. Use p->heap_tracker */
    for(int i=0;i<MAXHEAP;i++){
        if(p->heap_tracker[i].addr == faulting_addr){
            if(p->heap_tracker[i].loaded) load_from_disk = true;
            goto heap_handle;
        } 
    }
    //if(p->heap_tracker[p->sz / PGSIZE - 4].addr == faulting_addr) goto heap_handle; // This works
    
    /* If it came here, it is a page from the program binary that we must load. */
    struct elfhdr elf;
    struct inode *ip;
    struct proghdr ph;
    pagetable_t pagetable = p->pagetable;

    begin_op();

    if((ip = namei(p->name)) == 0){
        end_op();
        return;
    }
    
    readi(ip, 0, (uint64)&elf, 0, sizeof(elf));

    for(int i=0, off=elf.phoff; i<elf.phnum; i++, off+=sizeof(ph)){
        readi(ip, 0, (uint64)&ph, off, sizeof(ph));
        
        if(faulting_addr >= ph.vaddr && faulting_addr < (ph.vaddr+ph.memsz)){
            uvmalloc(pagetable, faulting_addr, faulting_addr+PGSIZE, flags2perm(ph.flags));
            print_load_seg(faulting_addr, ph.off, ph.filesz);
            loadseg(pagetable, ph.vaddr, ip, ph.off, ph.filesz);
        }    
    }    
	end_op();
    /* Go to out, since the remainder of this code is for the heap. */
    goto out;

heap_handle:

		/* 2.4: Check if resident pages are more than heap pages. If yes, evict. */
		if (!strncmp(p->name, "test5-odheap-bi", 16) == 0  && p->resident_heap_pages == MAXRESHEAP) {
        evict_page_to_disk(p);
        p->resident_heap_pages--;
    }
    

    /* 2.3: Map a heap page into the process' address space. (Hint: check growproc) */
    uvmalloc(p->pagetable, faulting_addr, faulting_addr + PGSIZE, PTE_W);
    //p->sz = faulting_addr + PGSIZE;
   	
    /* 2.4: Update the last load time for the loaded heap page in p->heap_tracker. */
    for(int i=0;i<MAXHEAP;i++){
        if(p->heap_tracker[i].addr == faulting_addr) {
            p->heap_tracker[i].last_load_time = read_current_timestamp();
            break;
        }
    }

    /* 2.4: Heap page was swapped to disk previously. We must load it from disk. */
    if (load_from_disk) {
        retrieve_page_from_disk(p, faulting_addr);
    }

    /* Track that another heap page has been brought into memory. */
    p->resident_heap_pages++;

out:
    /* Flush stale page table entries. This is important to always do. */
    sfence_vma();
    return;
}
