/*
 * file_manager.c
 *
 *  Created on: 8 maj 2020
 *      Author: stach
 */
#include "ff.h"
#include <stdlib.h>
#include <string.h>

char* GetNextFileName()
{
	 FRESULT res;
	 DIR dir;
	 static FILINFO fno;
	 char* path="/";

	 int ID=0;

	 res = f_opendir(&dir, path);                       /* Open the directory */
	 if (res == FR_OK) {
		 for (;;) {
			 res = f_readdir(&dir, &fno);                   /* Read a directory item */
			 if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			 else {                                       /* It is a file. */
				 if (ID < atoi(fno.fname))
				 {
					 ID = atoi(fno.fname);
				 }
			 }
		 }
		 f_closedir(&dir);
	 }

	 ID++;
	 char *c;
	 sprintf(c, "%d", ID);
	 strcat(c,".wav");
	 return c;
}

char * NextFile(char* file_name)
{
	FRESULT res;
	DIR dir;
	static FILINFO fno;
	char* path="/";

	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		 for (;;) {
			 res = f_readdir(&dir, &fno);                   /* Read a directory item */
			 if (res != FR_OK) break;  /* Break on error*/
			 if (!strcmp(file_name, fno.fname)){
				 char *prev = malloc(strlen(fno.name));
				 strcpy(prev, fno.name);
				 res = f_readdir(&dir, &fno);
				 if (res != FR_OK || fno.fname[0] == 0) break;
				 f_closedir(&dir);
				 return fno.fname;
			 }
		 }
		 f_closedir(&dir);
	 }
	 return prev;
}
char * PreviousFile(char* file_name) // to jeszcze do poprawki
{
	FRESULT res;
	DIR dir;
	static FILINFO fno;
	char* path="/";
	char* previous_name = malloc(strlen(file_name));

	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		 for (;;) {
			 previous_name = malloc(strlen(file_name));
			 res = f_readdir(&dir, &fno);                   /* Read a directory item */
			 if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			 if (!strcmp(file_name, fno.fname)){
				 res = f_readdir(&dir, &fno);
				 return previous_name;
			 }

		 }
		 f_closedir(&dir);
	 }
	 return fno.fname;
}
