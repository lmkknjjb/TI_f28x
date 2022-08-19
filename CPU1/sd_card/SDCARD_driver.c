/*
 * SD_CARD.c
//###########################################################################
// TI DSP F28377D for IMU, GPS NAVIGATION DAQ Algorithm V1.0
// by: Juhwan Lee, Konkuk University, GUNACO LAB.
// Date: Feb.20 2019
//###########################################################################
 */
#include "inc/Global_def.h"

extern FlagStatus F_FileOpen;
extern FlagStatus Q_data_write;

FATFS fs;            // Work area (file system object) for logical drive
DIR dirs;
FILINFO finfo;
FIL fsrc;      // file objects
FRESULT resOp;
char path[50] = {""};


void CreateNewFile(char * pch){
    Uint16 tmpNum = 0x0;
    char * pStr = pch + 2;
    const char Hname[] = "FD";
    char Fname[] = "FD";

    memcpy(Fname,pch,sizeof(char) * 2);

    if( strncmp(Fname, Hname, 2) ){
        char tmpch[] = "FD00000.txt";
        strcpy(pch,tmpch);
        printf("New File ... %s\n",pch);
        return ;
    }

    printf("Existed File ... %s \n",pch);
    //Increment
    tmpNum = atoi(pStr) + 1;
    printf("%d\n", tmpNum);
//    printf("%d",tmpNum);
    if(tmpNum > 30000)  DEF_RESETFLAG(tmpNum);

    //Naming
    sprintf(pch + 2, "%05d", tmpNum);
    strcat(pch, ".txt");
}


void SD_Initialize(char* pch){
  int dir_count = 0;

  f_mount(0,&fs);

  if( f_opendir(&dirs, path) == FR_OK ){
      while( f_readdir(&dirs, &finfo) == FR_OK ){
          dir_count ++;
          //      if ( finfo.fattrib & AM_DIR )   printf("\\%s\n", finfo.fname);

          if ( !(finfo.fattrib & AM_DIR) ){

              if(!finfo.fname[0])     break;

              strcpy(pch, finfo.fname);

          }
                if (dir_count == 10000){
                  break;
                }
          //            else
          //                break;
      }

    // Generating New File
    CreateNewFile(pch);
    printf("%s\n", pch);
    resOp = f_open(&fsrc, pch, FA_WRITE | FA_CREATE_ALWAYS);
    f_sync(&fsrc);

    printf("%s file is open!\n", pch);
    DEF_SETFLAG(F_FileOpen);


  }else{
    printf("Dir Open Failed ... \n");
    DEF_RESETFLAG(F_FileOpen);
  }
}

void SD_DataStorage(int16* pPack, Uint16 PackSize){
    UINT bytesWritten;
        f_write(&fsrc, pPack, PackSize, &bytesWritten);
}
//
void SD_Sync(void){
    f_sync(&fsrc);
}
//
//void SD_Close(void){
//    f_close (&fsrc);
//    f_mount(0, NULL);
//}
