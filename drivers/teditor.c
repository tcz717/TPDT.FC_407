#include <stdio.h>
#include <math.h>  
#include <stdlib.h>
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include <string.h>
#include <stm32f4xx.h>
#include <board.h>
#include <components.h>
#include <dfs_posix.h>

char input[100];
int script;
int cur;

static rt_err_t (*rx_indicate)(rt_device_t dev, rt_size_t size);
struct rt_semaphore rx_sem;
rt_device_t device ;

rt_err_t te_recv(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(&rx_sem);
	return RT_EOK;;
}

void take_console()
{
    device = rt_console_get_device();
	
	rx_indicate=device->rx_indicate;
	
	rt_sem_init(&rx_sem,"te_rx",0,RT_IPC_FLAG_FIFO);
	
	rt_device_set_rx_indicate(device,te_recv);
}
void release_console()
{
	rt_device_set_rx_indicate(device,rx_indicate);
	
	rt_sem_detach(&rx_sem);
}

int tgetc()
{
	char ch;
	rt_sem_take(&rx_sem,RT_WAITING_FOREVER);
	rt_device_read(device,0,&ch,1);
	return ch;
}
char * tgets(char * buf)
{
	int j=0;
	char ch;
	while((ch=tgetc())!='\n')
	{
		buf[j++]=ch;
	}
	buf[j]='\0';
	return buf;
}
int tscanf(const char * fmt, ...)
{
    va_list args;
	static char buf[128];
	
    va_start(args, fmt);
	
	tgets(buf);
	
	return vsscanf(buf,fmt,args);
}
int putc(int c, FILE * stream)
{
	rt_device_write(device,0,&c,1);	
}

void ShowText()
{
	char ch;
	int line = 1;
	lseek(script, 0, DFS_SEEK_SET);
	rt_kprintf("%c%3d ", line == cur ? '#' : ' ', line);
	while (read(script,&ch,1))
	{
		if (ch == '\n')
		{
			line ++ ;
			printf("\n%c%3d ", line == cur ? '#' : ' ', line);
			continue;
		}

		putchar(ch);
	}
}

void promt()
{
	rt_kprintf("\n>");
}

void parse(char * input)
{
	int size = strlen(input);

}

void Editor(char * path)
{
	take_console();
	
	cur = 1;
	script = open(path,O_APPEND|O_CREAT|O_RDWR,0);

	while (1)
	{
		ShowText();
		promt();

		tgets(input);
		printf("%s\n",input);

		parse(input);
	}

	close(script);
	
	release_console();
}

FINSH_FUNCTION_EXPORT_CMD(Editor,te,launch teditor)

