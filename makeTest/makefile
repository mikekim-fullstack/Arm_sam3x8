
thing_right := $(wildcard *.c)
all: f1 f2

.PHONY: f1
f1: 
	@echo $@

.PHONY: f2
f2: 
	@echo $@
# f1 f2:
# 	@echo $@
# 	cc -c hello.c -o hello1
# Equivalent to:
# f1.o:
#	 echo f1.o
# f2.o:
#	 echo f2.o
.PHONY:print
print: 
	@echo $(thing_right)