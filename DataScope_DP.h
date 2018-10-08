#ifndef _DATA_PRTOCOL_H_
#define _DATA_PRTOCOL_H_
 
extern unsigned char DataScope_OutPut_Buffer[42];    
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);   
unsigned char DataScope_Data_Generate(unsigned char Channel_Number); 
 
#endif 
