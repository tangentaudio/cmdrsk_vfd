# cmdrsk_vfd
LinuxCNC user component to control an Emerson/Control Techniques Commander SK VFD over RS485

Written by Steve Richardson (steve@tangentaudio.com) - December 2013
For LinuxCNC 2.6.0~pre

`gcc -I/usr/include -I/usr/include/linuxcnc -URTAPI -U__MODULE__ -DULAPI -Os  -o cmdrsk_vfd cmdrsk_vfd.c -Wl,-rpath,/lib -L/lib -llinuxcnchal -lmodbus -I/usr/include/modbus -llinuxcncini`


## Credits

> adapted from Michael Haberler's vfs11_vfd.c
>
>  based on: vfs11_vfd.c
>
>  Michael Haberler,  adapted from Steve Padnos' gs2_vfd.c, 
>  including modifications from John Thornton (jet1024 AT semo DOT net)
>
>  Copyright (C) 2007, 2008 Stephen Wille Padnos, Thoth Systems, Inc.
>  Copyright (C) 2009,2010,2011,2012 Michael Haberler
>
>  Based on a work (test-modbus program, part of libmodbus) which is
>  Copyright (C) 2001-2005 Stéphane Raimbault <stephane.raimbault@free.fr>

