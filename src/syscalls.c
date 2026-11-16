/* newlib只用于snprintf/sscanf。工程不使用文件系统或阻塞stdio。 */
#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>

extern char _end, _heap_limit;
void *_sbrk(ptrdiff_t increment) {
    static char *top;
    if (!top) top=&_end;
    if (increment<0 || increment>&_heap_limit-top) { errno=ENOMEM; return (void *)-1; }
    char *old=top; top+=increment; return old;
}
int _close(int fd) { (void)fd; errno=EBADF; return -1; }
int _fstat(int fd,struct stat *s) { (void)fd; s->st_mode=S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd,int offset,int origin) { (void)fd;(void)offset;(void)origin;errno=ESPIPE;return -1; }
int _read(int fd,char *buffer,int size) { (void)fd;(void)buffer;(void)size;errno=ENOSYS;return -1; }
int _write(int fd,const char *buffer,int size) { (void)fd;(void)buffer;(void)size;errno=ENOSYS;return -1; }
int _getpid(void) { return 1; }
int _kill(int pid,int signal) { (void)pid;(void)signal;errno=EINVAL;return -1; }
