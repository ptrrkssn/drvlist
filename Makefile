# Makefile for drvlist

OBJS=drvlist.o drive.o scsi.o ata.o nvme.o utils.o
LIBS=-lcam -lm

CFLAGS=-O -g -Wall

all: drvlist


scsi.h: drive.h
ata.h: drive.h
nvme.h: drive.h

drvlist.o: drvlist.c drive.h utils.h scsi.h ata.h nvme.h

drive.o: drive.c drive.h
utils.o: utils.c utils.h

scsi.o: scsi.c scsi.h 
ata.o: ata.c ata.h
nvme.o: nvme.c nvme.h



drvlist: $(OBJS)
	$(CC) -o drvlist $(OBJS) $(LIBS)

clean:
	rm -f drvlist *.o *~ core \#*

push:	clean
	git add -A && git commit -a && git push
