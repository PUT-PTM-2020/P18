/*
 * file_manager.c
 *
 *  Created on: 8 maj 2020
 *      Author: stach
 */
#include "ff.h"
#include <stdlib.h>

int GetNextFileID()
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
	 return ID;
}

FILINFO GetFileInfo(int song_number)
{
	FRESULT res;
	DIR dir;
	static FILINFO fno;
	char* path="/";

	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		 for (int i=0; i<song_number; i++) {
			 res = f_readdir(&dir, &fno);                   /* Read a directory item */
			 if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
		 }
		 f_closedir(&dir);
	 }
	 return fno;

}
