# Makefile for drvlist

OBJS=drvlist.o scsi.o ata.o nvme.o health.o smart.o tests.o utils.o
LIBS=-lcam -lm

CFLAGS=-O -g -Wall

all: drvlist


drvlist.o: drvlist.c health.h utils.h

scsi.o: scsi.c scsi.h health.h utils.h
ata.o: ata.c ata.h health.h utils.h
nvme.o: nvme.c nvme.h health.h utils.h

smart.o: smart.c smart.h scsi.h ata.h nvme.h utils.h
tests.o: tests.c tests.h scsi.h ata.h nvme.h utils.h

utils.o: utils.c utils.h


drvlist: $(OBJS)
	$(CC) -o drvlist $(OBJS) $(LIBS)

clean:
	rm -f drvlist *.o *~ core \#*

push:	clean
	git add -A && git commit -a && git push
