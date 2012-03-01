#ifndef _FASTBOOT_H_
#define _FASTBOOT_H_

#if defined CONFIG_FASTBOOT

void do_fastboot(void);
char *get_serial_number(void);

#else

void do_fastboot(void) { return; };
char *get_serial_number(void) { return 0; };

#endif /* CONFIG_FASTBOOT */

#endif /* FASTBOOT_H */
