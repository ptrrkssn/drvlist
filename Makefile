# Makefile for drvlist

OBJS=drvlist.o
LIBS=-lcam -lm

CFLAGS=-O -Wall

drvlist: $(OBJS)
	$(CC) -o drvlist $(OBJS) $(LIBS)

clean:
	rm -f drvlist *.o *~ core \#*

push:	clean
	git add -A && git commit -a && git push
