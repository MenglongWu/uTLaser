#ifndef _BOARD_H_
#define _BOARD_H_




extern int _where_lr();

extern int printk(const char *fmt, ...);

extern void usart_init();
extern int s_getchar(void);
extern int s_putchar(int data);
extern int s_peekchar(void);
extern int puts(const char *s);
extern void EINT_Handle();

//boot loader printf
#define bprintf(x) printk(x)




#endif


