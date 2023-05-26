//==========================================================================;
//  Copyright (C) 2000 PCO - Computer Optics GmbH.  All Rights Reserved.
//
//--------------------------------------------------------------------------;
/////////////////////////////////////////////////////////////////////////////
// definitions for dynamic link to Pixel_Fly drivers
// by MB, created with Visual C++ 6.0
/////////////////////////////////////////////////////////////////////////////
//for dynamic linking of DLL pccam.dll

HINSTANCE PccamLib=NULL;
int getpccamfunc(void);

//convert functions
HINSTANCE ConvertLib=NULL;
int getpcocnvfunc(void);

//pointer to camera functions
  int (*initboard)(int board,HANDLE *hdriver);
  int (*closeboard)(HANDLE *hdriver);
  int (*getboardpar)(HANDLE, unsigned int *buf, int len); //obsolete in version 2
  int (*getboardval)(HANDLE, int, void*); //only in version 2
  int (*getsizes)(HANDLE, int *ccdxsize, int *ccdysize,
                      int *actxsize, int *actysize,
                      int *bit_pix);
  int (*setmode)(HANDLE hdriver,int mode,
			               int explevel,int exptime,
			               int hbin,int vbin,
						   int gain,int offset,
						   int bit_pix,int shift);
  int (*start_camera)(HANDLE hdriver);
  int (*stop_camera)(HANDLE hdriver);
  int (*trigger_camera)(HANDLE hdriver);

//pointer to buffer functions
  int (*allocate_buffer)(HANDLE hdriver,int *bufnr,int *size);
  int (*free_buffer)(HANDLE hdriver,int bufnr);
  int (*getbuffer_status)(HANDLE hdriver,int bufnr,int mode,int *stat,int len);
  int (*add_buffer_to_list)(HANDLE hdriver,int bufnr,int size,int offset,int data);
  int (*remove_buffer_from_list)(HANDLE hdriver,int bufnr);
  int (*setbuffer_event)(HANDLE hdriver,int bufnr,HANDLE *hPicEvent);
  int (*map_buffer)(HANDLE hdriver,int bufnr,int size,int offset,DWORD *linadr); //for 32bit, dll-version 1 or higher
  int (*map_buffer_ex)(HANDLE hdriver, int bufnr, int size, int offset, void** linadr); //for 64bit, only dll-version 2 or higher
  int (*unmap_buffer)(HANDLE hdriver,int bufnr);

//pointer to convert functions
  void *(*create_bwlut)(int bitpix, int min_out, int max_out);
  int (*convert_set)(void *lut,int min,int max,int typ);
  int (*delete_bwlut)(void *lut);
  int (*conv_buf_12to8)(int mode,int width,int height,unsigned short *b12, unsigned char *b8,void *lut);
