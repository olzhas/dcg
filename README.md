Useful links
=============

http://man7.org/tlpi/code/online/diff/timers/ptmr_sigev_thread.c.html
http://stackoverflow.com/questions/26657744/is-it-possible-to-direct-linux-timer-notification-signal-to-a-specific-thread
http://man7.org/linux/man-pages/man2/sigaction.2.html
http://man7.org/linux/man-pages/man2/timer_create.2.html
http://man7.org/linux/man-pages/man7/pthreads.7.html

Build
=============
```
# mkdir build && cd build
# cmake ..
```

if gcc-4.7 is not your default compiler
```
# CC=gcc-4.7 cmake ..
```

Remarks
=============

max number of signals it is possible to send is
    SIGRTMAX - SIGRTMIN = 30
on raspbian
