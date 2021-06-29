// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/atomic.h>
#include <linux/vmalloc.h>
//#ifdef CONFIG_CMA
//#include <linux/cma.h>
//#endif
#include <asm/page.h>
#include <asm/pgtable.h>
//#include "internal.h"


static int saga_memInfo_proc_show(struct seq_file *m, void *v)
{
  char ufs_id[17];
  char *mem_ptr;
  char *mem_ptr_e;
  mem_ptr = strstr(saved_command_line, "ufs_id=");
  if (mem_ptr != 0) {
    mem_ptr_e = strstr(mem_ptr, " ");
    if (mem_ptr_e != 0) {
      strncpy(ufs_id, mem_ptr + 7,
          mem_ptr_e - mem_ptr - 7);
      ufs_id[mem_ptr_e - mem_ptr - 7] = '\0';
    }
    seq_printf(m, "%s\n", ufs_id);
  }
  return 0;
}

static int saga_memInfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, saga_memInfo_proc_show, NULL);
}

static const struct file_operations saga_memInfo_proc_fops = {
	.open		= saga_memInfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_saga_memInfo_init(void)
{
	proc_create("saga_memInfo", 0, NULL, &saga_memInfo_proc_fops);
	return 0;
}
fs_initcall(proc_saga_memInfo_init);
