/*******************************************************************************
  Universal Audio decoder demo.

  Company:
    Microchip Technology Inc.

  File Name:
    disk.c

  Summary:
   Contains the functional implementation of this demo application.

  Description:
   This file contains the functional implementation of this demo application.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"
#include "disk.h"
#include "audio_decoder/decoder.h"

static APP_AUDIOPLAYER *appDataPtr;
static DISK_TASK_DATA diskData;
static uint32_t bytes_read = 0;
extern uint16_t playerDiskDataSize;
static DISK_FILE_NODE rootNode;


// files index table
static DISK_FILE_PATH FilesTable[DISK_MAX_FILES];

#define TRAVERSE_DEPTH 5

void DISK_Initialize( void )
{
    diskData.state = DISK_STATE_REMOVED;

    
    rootNode.fstat.fattrib = SYS_FS_ATTR_DIR;
    strcpy(rootNode.path, SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0);
    strcat(rootNode.path, "/");
    appDataPtr = APP_GetAppDataInstance();
}

bool DISK_Mount()
{
   return true;
}

bool DISK_Unmount(void)
{
      if(SYS_FS_Unmount("/mnt/myDrive1") != 0)
      {
          /* The disk could not be mounted. Try
                 * mounting again untill success. */
          Nop();
          return false;
      }
      else
      {
          /* Mount was successful. Try opening the file */
          Nop();
          return true;
      }
}


//ListNode* DISK_Get_FileNameList()
//{
//    // get file names
//    ListNode *cur = root;
//    
//    int n = appDataPtr->totalAudioFiles;
//    int i=0;
////    if(cur == NULL)
////    {
////        //cur = cur->next; 
////        cur = malloc(sizeof(ListNode));
////        cur->val = FilesTable[0].path;
////       
////    }
//   
//    while(n--)
//    {
//        if(cur->next == NULL)
//        {
//            cur->next = malloc(sizeof(ListNode));
//        }
//        // extract the name
//        int k = 0;
//        int j;
//        char *path = FilesTable[i].path;
//        
//        for(j=0;j<64;j++)
//        {
//            if(path[j]=='/')
//            {
//                k = j;
//            }else if(path[j]=='.')
//            {
//                break;
//            }
//        }
//        for(j = k+1;j<64;j++)
//        {
//            if(path[j]!='\0')
//                (cur->next->val)[j-k-1] = path[j];
//            else
//                break;
//        }
//        cur->next->val[j] = '\0';
//        cur = cur->next;
//        i++;
//    }
//    return root->next;
//}

SYS_FS_RESULT DISK_FS_ReadDirFlat(const char *fname, uint8_t *dir_count, SYS_FS_FSTAT * dir_table, bool isRoot)
{
    SYS_FS_RESULT ret;
    diskData.dirHandle = SYS_FS_DirOpen(fname);
   
    if(diskData.dirHandle == SYS_FS_HANDLE_INVALID)
    {
        return SYS_FS_RES_FAILURE;
    }
    *dir_count = 0;
    
    do
     {
         if(appDataPtr->totalAudioFiles < DISK_MAX_FILES)
         {
             diskData.dirstat.fname[0] = '\0';
             ret = SYS_FS_DirRead(diskData.dirHandle,&diskData.dirstat);

             // End of this directory
             if(diskData.dirstat.fname[0] == '\0')
             {
                 break;
             }
             if(ret!= SYS_FS_RES_FAILURE)
             {

                 if(diskData.dirstat.fattrib != SYS_FS_ATTR_DIR)
                 {
                     
                    if(APP_IsSupportedAudioFile(diskData.dirstat.fname))
                    {
                        strcpy(FilesTable[appDataPtr->totalAudioFiles].path, fname);
                        if(!isRoot)
                        {
                            strcat(FilesTable[appDataPtr->totalAudioFiles].path, "/");
                        }
                        strcat(FilesTable[appDataPtr->totalAudioFiles].path, diskData.dirstat.fname);
                        
                        (appDataPtr->totalAudioFiles)++;
                    }
                     
                 }else if(diskData.dirstat.fattrib == SYS_FS_ATTR_DIR && diskData.dirstat.fname[0] != '.') // Skip ".\" and "..\" directories
                 {
                     if(*dir_count < DISK_MAX_DIRS)
                     {
                         dir_table[*dir_count]=diskData.dirstat;
                         (*dir_count)++;
                     }
                 }
             }else{
                ret = SYS_FS_RES_FAILURE;
                break;
             }
         }
         else
         {
             ret = SYS_FS_RES_FAILURE;
             break;
         }
     }while(ret==SYS_FS_RES_SUCCESS);
     
     SYS_FS_DirClose(diskData.dirHandle);
     return ret;
}

//static bool isRootDir(char * dir)
//{
//    if(strcmp(dir, SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0)==0)
//    {
//        return true;
//    }
//    return false;
//}
//bool DISK_isCurrentPathRoot()
//{
//    return isRootDir(FilesTable[appDataPtr->currentSongIdx].path);
//}

void DISK_TraverseAllFiles(DISK_FILE_NODE node, bool isRoot,uint8_t depth){
     if(depth > TRAVERSE_DEPTH)
     {
         return;
     }
     int i;
     SYS_FS_RESULT ret;

     uint8_t totalDir = 0;
     SYS_FS_FSTAT dirTable[DISK_MAX_DIRS];
     
     ret = DISK_FS_ReadDirFlat(node.path, &totalDir, dirTable, isRoot);

     if(ret == SYS_FS_RES_FAILURE)
     {
//         SYS_FS_DirClose(diskData.dirHandle);
         return;
     }

     DISK_FILE_NODE child_node;
     
     for(i = 0; i < totalDir; i++)
     {
         
         child_node.fstat = dirTable[i];
         
         strcpy(child_node.path, node.path);
         if(!isRoot)
         {
             strcat(child_node.path, "/");
         }
         strcat(child_node.path, child_node.fstat.fname);
         DISK_TraverseAllFiles(child_node, false, depth+1);
         
     }
     
     return;
 }
 

bool DISK_ScanTask(void )
{
    return true;
}

void DISK_Tasks()
{
    USB_Connection_Tasks();
    
    if(appDataPtr->state==APP_STATE_DEVICE_CONNECTED)
    {
        if (diskData.state == DISK_STATE_REMOVED )
        {
            diskData.state = DISK_STATE_INIT;
            DISK_EventHandler ( DISK_EVENT_INSERTED, 0 ,appDataPtr->fileHandle);
        }
    }
    else
    {
        if ( diskData.state != DISK_STATE_REMOVED )
        {
            diskData.state = DISK_STATE_REMOVED;
            DISK_EventHandler ( DISK_EVENT_REMOVED, 0 ,appDataPtr->fileHandle);
        }
    
    }
    switch ( diskData.state )
    {
        case DISK_STATE_HALT:
            break;

        case DISK_STATE_REMOVED:
            appDataPtr->isDiskMounted = false;
            break;

        case DISK_STATE_INIT:
            diskData.state = DISK_STATE_SCANNING;
            diskData.scanstate = DISK_SCAN_OPEN_DIR;

            appDataPtr->totalAudioFiles = 0;
            appDataPtr->currentSongIdx  = 0;
            appDataPtr->nextSongIdx     = 0;
            appDataPtr->previousSongIdx = 0;

            appDataPtr->isDiskMounted = true;
            DISK_EventHandler ( DISK_EVENT_SCANNING_STARTED, 0, appDataPtr->fileHandle);
        
            break;

        case DISK_STATE_SCANNING:
            DISK_TraverseAllFiles(rootNode, true, 0);
            
            if(appDataPtr->totalAudioFiles == 0){
                // No Audio File
                diskData.state = DISK_STATE_NO_AUDIO_FILES;
            }else{
                appDataPtr->currentSongIdx = 0;
//                appDataPtr->playerState = APP_STATE_RUNNING;
                diskData.state = DISK_STATE_SCAN_FINISHED;
                DISK_EventHandler ( DISK_EVENT_SCANNING_FINISHED, 0, 0);  //from release
                
                // must check codec status state, player is ready to stream audio
                appDataPtr->playerState = APP_STATE_CODEC_OPEN;
            }
            break;
        
        case DISK_STATE_SCAN_FINISHED:
            if(appDataPtr->playerState != APP_STATE_CODEC_OPEN){
                diskData.state = DISK_STATE_OPEN_FIRST_TRACK;
            }
            break;
        case DISK_STATE_OPEN_FIRST_TRACK:
            if(DISK_OpenTrack(FilesTable[0].path))
//            //if(DISK_OpenTrack("wt.wav"))  //Not in release
            {
                APP_PlayerEventHandler ( PLAYER_EVENT_READY, appDataPtr->totalAudioFiles );
                appDataPtr->playerState = APP_STATE_RUNNING;
            }
            diskData.state = DISK_STATE_RUNNING;
            break;
        
    
       
        case DISK_STATE_RUNNING:
            break;

        case DISK_STATE_NO_AUDIO_FILES:
            appDataPtr->playerState = APP_STATE_NO_FILE;
            diskData.state = DISK_STATE_RUNNING;
            APP_PlayerEventHandler(PLAYER_EVENT_NO_FILES_FOUND, (uint32_t)"No Audio File Found" );
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////

bool DISK_NextTrack(void)
{
    if(appDataPtr->currentSongIdx == appDataPtr->totalAudioFiles-1)
    {
        appDataPtr->nextSongIdx = 0;
//        strcpy(diskData.curFilePath, FilesTable[appDataPtr->nextSongIdx].path);
    }else{
        appDataPtr->nextSongIdx = appDataPtr->currentSongIdx + 1;
//        strcpy(diskData.curFilePath, FilesTable[appDataPtr->nextSongIdx].path);
    }
    
    appDataPtr->currentSongIdx = appDataPtr->nextSongIdx;

    if (DISK_OpenTrack(FilesTable[appDataPtr->currentSongIdx].path) == true)
    {
        return true;
    }
    else
        return false;
}

bool DISK_PreviousTrack()
{
    appDataPtr->previousSongIdx = (appDataPtr->currentSongIdx ? (appDataPtr->currentSongIdx - 1):(appDataPtr->totalAudioFiles - 1));
    appDataPtr->currentSongIdx = appDataPtr->previousSongIdx;
    if (DISK_OpenTrack(FilesTable[appDataPtr->currentSongIdx].path) == true)
    {
        return true;
    }
    else
        return false;
}

////////////////////////////////////////////////////////////////////////////////

bool DISK_OpenTrack ( const char *fname )
{
    appDataPtr->fileHandle = DISK_OpenFile(fname);

    if ( appDataPtr->fileHandle != SYS_FS_HANDLE_INVALID )
    {
        
        if(DISK_FileStatus(fname) == true)
        {
            appDataPtr->current_filesize = DISK_GetFileSize(appDataPtr->fileHandle);
            DISK_EventHandler ( DISK_EVENT_TRACK_CHANGED, appDataPtr->diskCurrentFile, appDataPtr->fileHandle );
            
            bytes_read = DISK_GetFilePosition(appDataPtr->fileHandle);
            return true;
        }else
        {
              return false;
        }
        
    }
    else
    {
        DISK_EventHandler ( DISK_EVENT_FILE_OPEN_ERROR, appDataPtr->diskCurrentFile,appDataPtr->fileHandle );
        return ( false );
    }
    
    return true;
}

bool DISK_ReopenTrack(void)
{
    if (DISK_OpenTrack(FilesTable[appDataPtr->currentSongIdx].path) == true)
    {
        return true;
    }
    else
        return false;
}


SYS_FS_HANDLE DISK_OpenFile ( const char *fname )
{
    SYS_FS_HANDLE fileHandle;
//    if ( diskData.state != DISK_STATE_RUNNING && diskData.state != DISK_STATE_NO_AUDIO_FILES && diskData.state != DISK_STATE_SCANNING)
//    {
//        return ( SYS_FS_HANDLE_INVALID );
//    }

    bytes_read = 0;
    playerDiskDataSize=0;

    fileHandle = SYS_FS_FileOpen(fname, (SYS_FS_FILE_OPEN_READ_PLUS));
    return fileHandle;
}

 bool DISK_CloseFile (SYS_FS_HANDLE fileHandle )
{
    SYS_FS_RESULT ret;
    if ( fileHandle != SYS_FS_HANDLE_INVALID )
    {
        ret = SYS_FS_FileClose ( fileHandle );
        if(ret == SYS_FS_RES_SUCCESS)
        {
            return true;
        }
    }
    return false;
}

uint16_t DISK_GetTotalFiles ( void )
{
    return ( appDataPtr->totalAudioFiles );
}

uint16_t DISK_GetCurrentFileNumber ( void )
{
    return ( (uint16_t) appDataPtr->diskCurrentFile );
}

bool DISK_FileNameGet(SYS_FS_HANDLE handle, char* cName)
{
    bool stat = SYS_FS_FileNameGet(handle, (uint8_t*) cName, 255);
    return stat;
}

bool DISK_FileStatus(const char* fname)
{
     if( SYS_FS_FileStat(fname, &(appDataPtr->fileStatus)) == SYS_FS_RES_FAILURE )
     {
        return false;
     }
     
     uint8_t len = 0;
    
     if (appDataPtr->fileStatus.lfname[0]=='\0') // name is shorter than 8 char
     {
        len = strlen(appDataPtr->fileStatus.fname);
        if (len > 0)
        {
            strncpy (appDataPtr->fileStatus.lfname, appDataPtr->fileStatus.fname, len);
            appDataPtr->fileStatus.lfname[len] = '\0'; // null character
        }
     }
     
     return true;
}

uint32_t DISK_GetFileSize(SYS_FS_HANDLE fileHandle)
{
    appDataPtr->fileSize = SYS_FS_FileSize( fileHandle );

    if(appDataPtr->fileSize == -1)
    {
        /* Reading file size was a failure */
        return (uint32_t) SYS_FS_HANDLE_INVALID;
    }
    else
    {
        return (uint32_t)appDataPtr->fileSize;
    }
}

bool DISK_FSeek(SYS_FS_HANDLE fileHandle,int32_t offset)
{
    if(SYS_FS_FileSeek( fileHandle, offset, SYS_FS_SEEK_SET ) == -1)
    {
         return false;
    }
    else
    {
        /* Check for End of file */
         return true;
    }
}

bool DISK_EndOfFile(void)
{
    
    if(SYS_FS_FileEOF( appDataPtr->fileHandle ) == false )
    {
        return false;
    }
    else
    {
        return true;
    }

}
bool DISK_SeekStartLocation(SYS_FS_HANDLE fileHandle,int32_t offset)
{
    /* Move file pointer to begining of file */
    if(SYS_FS_FileSeek( fileHandle, offset, SYS_FS_SEEK_END ) == -1)
    {
            return false;
    }
    else
    {
            return true;
    }
            
}

bool DISK_FillBuffer(uint8_t *ptr)
{
    switch(appDataPtr->currentStreamType)
    {
        case APP_STREAM_MP3:
#ifdef MP3_DECODER_ENABLED
        {
            if(bytes_read < appDataPtr->current_filesize )
            {

                while(playerDiskDataSize < (MP3_DECODER_INPUT_FRAME_SIZE)) //1538 bytes MP3_PLAYER_INPUT_BUFFER_SIZE - MEDIA_SECTOR_SIZE
                { 
                    appDataPtr->nBytesRead = SYS_FS_FileRead(appDataPtr->fileHandle,ptr+playerDiskDataSize,(MEDIA_SECTOR_SIZE));

                    if ((appDataPtr->nBytesRead == -1) ||(DISK_EndOfFile() == true ))
                    {
                        bytes_read = 0;
                        SYS_FS_FileClose(appDataPtr->fileHandle);
                        return false;
                    }
                    else
                    {
                        bytes_read += appDataPtr->nBytesRead;
                        playerDiskDataSize += appDataPtr->nBytesRead;
                        appDataPtr->nBytesRead =0;

                    }
                }
            }else
            {

                bytes_read = 0;
                return false;
            }
        }
#endif
        break;
        case APP_STREAM_SPEEX:
        case APP_STREAM_OPUS:
#if defined(OGG_SPEEX_DECODER_ENABLED) || defined(OGG_OPUS_DECODER_ENABLED)
        {
            if(bytes_read < appDataPtr->current_filesize){
            // read packet by packet  
                if(appDataPtr->currentStreamType == APP_STREAM_SPEEX)
                {
#ifdef OGG_SPEEX_DECODER_ENABLED
                    appDataPtr->nBytesRead = SPEEX_DiskRead(ptr);//DECODER_MAX_INPUT_BUFFER_SIZE
#endif
                }
                else
                {
#ifdef OGG_OPUS_DECODER_ENABLED
                    appDataPtr->nBytesRead = OPUS_DiskRead(ptr);//DECODER_MAX_INPUT_BUFFER_SIZE
#endif
                }

                if ((appDataPtr->nBytesRead == -1))
                {
                    bytes_read = 0;
                    SYS_FS_FileClose(appDataPtr->fileHandle);
                    return false;
                }
                else
                {
                    bytes_read += appDataPtr->nBytesRead;
                    playerDiskDataSize += appDataPtr->nBytesRead;
                    appDataPtr->nBytesRead =0;
                }
            }else{ // end of file
                bytes_read = 0;
                SYS_FS_FileClose(appDataPtr->fileHandle);           
                return false;
            }
        }
#endif
        break;
        case APP_STREAM_FLAC:
#ifdef FLAC_DECODER_ENABLED
        {
            appDataPtr->readBytes = FLAC_GetBlockSize();
        
            if ((appDataPtr->readBytes == -1))
            {
                bytes_read = 0;
                SYS_FS_FileClose(appDataPtr->fileHandle);
                return false;
            } 
            else
            {
                return true;
            }
        }
#endif
        break;
        
        case APP_STREAM_WMA:
        case APP_STREAM_WAV:
        case APP_STREAM_AAC:
        case APP_STREAM_ADPCM:
        {
            if((bytes_read <appDataPtr->current_filesize))
            { 
                if((appDataPtr->readbyte_flag))
                {
                    appDataPtr->nBytesRead = SYS_FS_FileRead( appDataPtr->fileHandle,ptr,appDataPtr->readBytes );
                    if ((appDataPtr->nBytesRead == -1)||(DISK_EndOfFile() == true ))
                    {
                        bytes_read = 0;
                        SYS_FS_FileClose(appDataPtr->fileHandle);
                        return false;
                    }
                    else
                    {
                        bytes_read += appDataPtr->nBytesRead;
                        playerDiskDataSize += appDataPtr->nBytesRead;
                        appDataPtr->nBytesRead =0;
                    }
                }
            }else
            {
                bytes_read = 0;
                return false;
            }
        }
        
        break;
        default:
            break;
    }
    
    return true;

//    if(appDataPtr->currentStreamType == APP_STREAM_MP3)
//    {
//        if(bytes_read < appDataPtr->current_filesize )
//        {
//            
//            while(playerDiskDataSize < (MP3_DECODER_INPUT_FRAME_SIZE)) //1538 bytes MP3_PLAYER_INPUT_BUFFER_SIZE - MEDIA_SECTOR_SIZE
//            { 
//                appDataPtr->nBytesRead = SYS_FS_FileRead(appDataPtr->fileHandle,ptr+playerDiskDataSize,(MEDIA_SECTOR_SIZE));
//                
//                if ((appDataPtr->nBytesRead == -1) ||(DISK_EndOfFile() == true )) //
//                {
//                    bytes_read = 0;
//                    SYS_FS_FileClose(appDataPtr->fileHandle);
//                    return false;
//                }
//                else
//                {
//                    bytes_read += appDataPtr->nBytesRead;
//                    playerDiskDataSize += appDataPtr->nBytesRead;
//                    appDataPtr->nBytesRead =0;
//                    
//                }
//            }
//        }else
//        {
//          
//            bytes_read = 0;
//            return false;
//        }
//
//    }else if(appDataPtr->currentStreamType == APP_STREAM_SPEEX || appDataPtr->currentStreamType == APP_STREAM_OPUS){
//        if(bytes_read < appDataPtr->current_filesize){
//            // read packet by packet  
//            if(appDataPtr->currentStreamType == APP_STREAM_SPEEX)
//            {
//                appDataPtr->nBytesRead = SPEEX_DiskRead(ptr);//DECODER_MAX_INPUT_BUFFER_SIZE
//            }else{
//                appDataPtr->nBytesRead = OPUS_DiskRead(ptr);//DECODER_MAX_INPUT_BUFFER_SIZE
//            }
//            
//            if ((appDataPtr->nBytesRead == -1))
//            {
//                bytes_read = 0;
//                SYS_FS_FileClose(appDataPtr->fileHandle);
//                return false;
//            }
//            else
//            {
//                bytes_read += appDataPtr->nBytesRead;
//                playerDiskDataSize += appDataPtr->nBytesRead;
//                appDataPtr->nBytesRead =0;
//            }
//        }else{ // end of file
//            bytes_read = 0;
//            SYS_FS_FileClose(appDataPtr->fileHandle);           
//            return false;
//        }
//    }
//    /* Tobe added   */
//    else if(appDataPtr->currentStreamType == APP_STREAM_FLAC ) {
//        /**/
//        appDataPtr->readBytes = FLAC_GetBlockSize();
//        
//        if ((appDataPtr->readBytes == -1))
//        {
//            bytes_read = 0;
//            SYS_FS_FileClose(appDataPtr->fileHandle);
//            return false;
//        } 
//        else
//        {
//
//            return true;
////            bytes_read += appDataPtr->nBytesRead; 
////            playerDiskDataSize += appDataPtr->nBytesRead;
////            appDataPtr->nBytesRead = 0;
//        }
//    }
//    else 
//    {
//        if((bytes_read <appDataPtr->current_filesize))
//        { 
//            if((appDataPtr->readbyte_flag))
//            {
//                appDataPtr->nBytesRead = SYS_FS_FileRead( appDataPtr->fileHandle,ptr,appDataPtr->readBytes );
//                if ((appDataPtr->nBytesRead == -1)||(DISK_EndOfFile() == true ))
//                {
//                    bytes_read = 0;
//                    SYS_FS_FileClose(appDataPtr->fileHandle);
//                    return false;
//                }
//                else
//                {
//                    bytes_read += appDataPtr->nBytesRead;
//                    playerDiskDataSize += appDataPtr->nBytesRead;
//                    appDataPtr->nBytesRead =0;
//                }
//            }
//        }else
//        {
//            bytes_read = 0;
//            return false;
//        }
//    }
//    return true;
}


int32_t DISK_GetFilePosition (SYS_FS_HANDLE fileHandle )
{
    int32_t tell;

    tell = SYS_FS_FileTell(fileHandle);
    return ( tell );
}

int32_t DISK_GetCurrentFilePosition(){
    return SYS_FS_FileTell(appDataPtr->fileHandle);
}

bool DISK_SetFilePosition (SYS_FS_HANDLE fileHandle, int32_t pos )
{
    int status;
    status = SYS_FS_FileSeek(fileHandle, pos, SYS_FS_SEEK_SET);
    if( status == pos )
    {
        return true;
    }
    return false;
}

bool DISK_SetCurrentFilePosition(int32_t pos)
{
    int status;
    status = SYS_FS_FileSeek(appDataPtr->fileHandle, pos, SYS_FS_SEEK_SET);
    if( status == pos )
    {
        return true;
    }
    return false;
}

bool DISK_SetCurrentFilePositionWithControl(int32_t pos, SYS_FS_FILE_SEEK_CONTROL whence)
{
    int status;
    status = SYS_FS_FileSeek(appDataPtr->fileHandle, pos, whence);
    if( status == pos )
    {
        return true;
    }
    return false;
}


uint32_t DISK_ReadCurrentFile(uint8_t *ptr, size_t readSize){
    uint32_t ret = 0;
    appDataPtr->nBytesRead = SYS_FS_FileRead( appDataPtr->fileHandle,ptr,readSize);
    if ((appDataPtr->nBytesRead != readSize))
    {
        ret = 0;
        bytes_read = 0;
    }
    else
    {
        appDataPtr->nBytesRead =0;
        return readSize;
    }
    
    return ret;
}

uint32_t DISK_GetCurrentFileSize()
{
//    if(appDataPtr->fileSize != 0)
//        return appDataPtr->fileSize;
    
    appDataPtr->fileSize = SYS_FS_FileSize( appDataPtr->fileHandle );
    if(appDataPtr->fileSize == -1)
    {
        /* Reading file size was a failure */
        return (uint32_t) SYS_FS_HANDLE_INVALID;
    }
    else
    {
        return (uint32_t)appDataPtr->fileSize;
    }
}
