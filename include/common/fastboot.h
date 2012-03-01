#ifndef _FASTBOOT_H_
#define _FASTBOOT_H_

/* Write the file as a series of variable/value pairs
   using the setenv and saveenv commands */
#define FASTBOOT_PTENTRY_FLAGS_WRITE_ENV	0x00000400

/* Android-style flash naming */
typedef struct fastboot_ptentry fastboot_ptentry;

/* flash partitions are defined in terms of blocks
** (flash erase units)
*/
struct fastboot_ptentry {
	/* The logical name for this partition, null terminated */
	char name[16];
	/* The start wrt the nand part, must be multiple of nand block size */
	unsigned int start;
	/* The length of the partition, must be multiple of nand block size */
	u64 length;
	/* Controls the details of how operations are done on the partition
	See the FASTBOOT_PTENTRY_FLAGS_*'s defined below */
	unsigned int flags;
};

#if defined CONFIG_FASTBOOT

void do_fastboot(void);
char *get_serial_number(void);

void fastboot_flash_reset_ptn(void);
void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count);
void fastboot_flash_dump_ptn(int count);
unsigned int fastboot_flash_get_ptn_count(void);
fastboot_ptentry *fastboot_flash_find_ptn(const char *name);

extern int fastboot_oem(void);
extern char *get_ptn_size(char *buf, const char *ptn);

#else

void do_fastboot(void) { return; };
char *get_serial_number(void) { return 0; };

void fastboot_flash_reset_ptn(void) { return; };
void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count) { return; };
void fastboot_flash_dump_ptn(int count) { return; };
unsigned int fastboot_flash_get_ptn_count(void) { return 0; };
fastboot_ptentry *fastboot_flash_find_ptn(const char *name) { return; };

extern int fastboot_oem(void) { return 0; }
extern char *get_ptn_size(char *buf, const char *ptn) { return 0; };

#endif /* CONFIG_FASTBOOT */

#endif /* FASTBOOT_H */
