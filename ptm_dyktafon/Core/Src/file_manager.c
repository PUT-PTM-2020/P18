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
    res = f_opendir(&dir, path);                       /* Open the directory */
    if (atoi(file_name)==0)
    {
        res = f_readdir(&dir, &fno);
        if (res)
        {
      	  	LCD1602_clear();
      	  	LCD1602_1stLine();
      		LCD1602_print("NO CARD");
      		HAL_Delay(10000);
        }
        res = f_readdir(&dir, &fno);
        f_closedir(&dir);
        if(fno.fname[0]!=0)
        {
        char *c;
		c=malloc(strlen(fno.fname));
		strcpy(c,fno.fname);
		 return c;
        }
        else return file_name;
    }
    if (res == FR_OK) {
         for (;;) {
             res = f_readdir(&dir, &fno);                   /* Read a directory item */
             if (res != FR_OK) break;  /*Break on error*/
             int string1 = atoi(file_name);
             int string2 = atoi(fno.fname);
             if (string1==string2){
                 res = f_readdir(&dir, &fno);

                 if (res != FR_OK || fno.fname[0] == 0) break;
                 f_closedir(&dir);

              char *c;
			  c=malloc(strlen(fno.fname));
			  strcpy(c,fno.fname);
			  return c;
             }
         }
         f_closedir(&dir);
     }
     return file_name;
}


char * PreviousFile(char* file_name) // to jeszcze do poprawki
{
   FRESULT res;
   DIR dir;
   static FILINFO fno;
   char* path="/";
   char* previous_name = malloc(sizeof(char)*strlen(file_name));
   strcpy(previous_name,file_name);
   if (atoi(file_name)==0)
   {
      return file_name;
   }
   res = f_opendir(&dir, path);                       /* Open the directory */

   if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (atoi(file_name)==atoi(fno.fname)){
                if(previous_name[0]=='S' )
                {return file_name;}
            	return previous_name;
            }
            free(previous_name);
            previous_name = malloc(strlen(file_name));
            strcpy(previous_name,fno.fname);

        }
        f_closedir(&dir);
    }
   if(previous_name[0]=='S' )
   {return file_name;}
	return previous_name;
}


