/*
 * file_manager.c
 *
 *  Created on: 8 maj 2020
 *      Author: stach
 */
#include "ff.h"
#include <stdio.h>
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
	 c=malloc(8);
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
	char * prev;
	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		 for (;;) {
			 res = f_readdir(&dir, &fno);                   /* Read a directory item */
			 if (res != FR_OK) break;  /* Break on error*/
			 if (atoi(file_name)==atoi(fno.fname)){
				 prev = malloc(strlen(fno.fname));
				 strcpy(prev, fno.fname);
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
   char* previous_name = malloc(sizeof(char)*strlen(file_name));
   strcpy(previous_name,file_name);

   res = f_opendir(&dir, path);                       /* Open the directory */
   if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (atoi(file_name)==atoi(fno.fname)){

                return previous_name;
            }
            free(previous_name);
            previous_name = malloc(strlen(file_name));
            strcpy(previous_name,fno.fname);

        }
        f_closedir(&dir);
    }
    return previous_name;
}

char * PreviousFilestare(char* file_name) // to jeszcze do poprawki
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
