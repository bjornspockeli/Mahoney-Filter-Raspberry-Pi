### MAKEFILE ###
CC	:=gcc
CFLAGS	:= -Wall
INCLUDE	:= usr/local/include 
LIBS	:= -lwiringPi -lm
DEPS 	:= math.h stdio.h wiringPi.h  

EXE 	:= mahoney
SRCDIR 	:= src
BUILDDIR:= build

SRC 	:= $(wildcard $(SRCDIR)/*.c)
OBJ 	:= $(SRC:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)


$(EXE): $(OBJ)
	$(CC) $^ -o $@ -L $(INCLUDE) $(LIBS)

$(BUILDDIR)/%.o: $(SRCDIR)/%.c #$(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@   

clean:
	rm $(OBJ)
