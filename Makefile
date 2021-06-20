CC=git


all: addfirst addmake commit push

clone:
	$(CC) clone https://github.com/99004481-TadiboyinaMuralikrishna/test.git

addfirst:
	$(CC) add STM32_Project
addmake:
	$(CC) add Makefile
	
commit:
	$(CC) commit -m 'done' 

push:
	$(CC)  push origin main 

