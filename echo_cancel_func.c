
/*********************************************
		simulate-for-perf.

********************************************/
 #include <rtdevice.h>
 #include <math.h>
 #include <stdio.h>
 
/***********************************************************
		pcm-voice-data to linear

***********************************************************/
short  pcm2Linear(unsigned char pcm_data)
{
		unsigned char pcm_temp;
		unsigned char seg_value;
		unsigned short linear_data;
		pcm_temp = (~pcm_data) & 0x55;
		pcm_data = pcm_temp | (pcm_data & 0xaa);
		seg_value = (pcm_data&0x70)>>4;		
		if (seg_value == 0){
			linear_data = pcm_data&0x0f;
		}else if(seg_value >=1 && seg_value <= 7){	
			linear_data = (((unsigned short)8) << seg_value) + ((pcm_data&0x0f) << (seg_value-1));
		}
		if ((pcm_data & 0x80) == 0){
			  linear_data = -linear_data;	
		}
		linear_data = linear_data >> 4;
		return linear_data;
}
/************************************************************
		desp:  pcm-to-linear/vice vose


*************************************************************/
const int MAX = 32635; //32767 (max 15-bit integer) minus BIAS
/*A-law to linear */
static char encode(int pcm)
{
    //Get the sign bit. Shift it for later use 
    //without further modification
    int sign = (pcm & 0x8000) >> 8;
    //If the number is negative, 
    //make it positive (now it's a magnitude)
    if (sign != 0)
        pcm = -pcm;
    //The magnitude must fit in 15 bits to avoid overflow
    if (pcm > MAX) pcm = MAX;

    /* Finding the "exponent"
     * Bits:
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * S 7 6 5 4 3 2 1 0 0 0 0 0 0 0 0
     * We want to find where the first 1 after the sign bit is.
     * We take the corresponding value 
     * from the second row as the exponent value.
     * (i.e. if first 1 at position 7 -> exponent = 2)
     * The exponent is 0 if the 1 is not found in bits 2 through 8.
     * This means the exponent is 0 even if the "first 1" doesn't exist.
     */
    int exponent = 7;
    //Move to the right and decrement exponent 
    //until we hit the 1 or the exponent hits 0
    for (int expMask = 0x4000; (pcm & expMask) == 0 
         && exponent>0; exponent--, expMask >>= 1) { }

    /* The last part - the "mantissa"
     * We need to take the four bits after the 1 we just found.
     * To get it, we shift 0x0f :
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * S 0 0 0 0 0 1 . . . . . . . . . (say that exponent is 2)
     * . . . . . . . . . . . . 1 1 1 1
     * We shift it 5 times for an exponent of two, meaning
     * we will shift our four bits (exponent + 3) bits.
     * For convenience, we will actually just
     * shift the number, then AND with 0x0f. 
     * 
     * NOTE: If the exponent is 0:
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * S 0 0 0 0 0 0 0 Z Y X W V U T S (we know nothing about bit 9)
     * . . . . . . . . . . . . 1 1 1 1
     * We want to get ZYXW, which means a shift of 4 instead of 3
     */
    int mantissa = (pcm >> ((exponent == 0) ? 4 : (exponent + 3))) & 0x0f;

    //The a-law byte bit arrangement is SEEEMMMM 
    //(Sign, Exponent, and Mantissa.)
    char alaw = (char)(sign | exponent << 4 | mantissa);

    //Last is to flip every other bit, and the sign bit (0xD5 = 1101 0101)
    return (char)(alaw^0xD5);
}

static short decode(unsigned char alaw)
{
    //Invert every other bit, 
    //and the sign bit (0xD5 = 1101 0101)
    alaw ^= 0xD5;

    //Pull out the value of the sign bit
    int sign = alaw & 0x80;
    //Pull out and shift over the value of the exponent
    int exponent = (alaw & 0x70) >> 4;
    //Pull out the four bits of data
    int data = alaw & 0x0f;

    //Shift the data four bits to the left
    data <<= 4;
    //Add 8 to put the result in the middle 
    //of the range (like adding a half)
    data += 8;    

		//If the exponent is not 0, then we know the four bits followed a 1,
    //and can thus add this implicit 1 with 0x100.
    if (exponent != 0){
        data += 0x100;
		}	
    /* Shift the bits to where they need to be: left (exponent - 1) places
     * Why (exponent - 1) ?
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * . 7 6 5 4 3 2 1 . . . . . . . . <-- starting bit (based on exponent)
     * . . . . . . . Z x x x x 1 0 0 0 <-- our data (Z is 0 only when <BR>     * exponent is 0)
     * We need to move the one under the value of the exponent,
     * which means it must move (exponent - 1) times
     * It also means shifting is unnecessary if exponent is 0 or 1.
     */
    if (exponent > 1){
        data <<= (exponent - 1);
		}
    return (short)(sign == 0 ? data : -data);
}


/*****************************************************
	description: 	huxiaofei-write
			 status:  debuging, maybe-modifing-later
	far_end_addr:  down-voice
	near_end_addr: up-voice

*****************************************************/

#define  DTD 
#define  NLMS
#define  ORDER  50 //20 50 100 128 180 220
//#define  DEBUG_ECHO

//pcm-tranfer-to-linear-buffer
short down_Buff_linear[12*16]	={0};
short   up_Buff_linear[12*16]	={0};
short 	processed_addr[12*16] ={0};

//debug
static int prt_cnt = 0;
#if 0
const float u = 0.000000000005; 			// 0.0000000005;
short* far_end_addr 	= (short*) down_Buff_linear;
short* near_end_addr 	= (short*) up_Buff_linear;	 
		
short far_end_voice = 0;
short near_end_voice = 0;
float far_end_voice_f = 0.0f;
float near_end_voice_f = 0.0f;

float e = 0.0f;	//err
float y = 0.0f;	//sum-result 

static float far_end_voice_array[2][ORDER] = {1};
static float w[2][ORDER] = {0};

#ifdef DTD
		static int max_window_cnt[2] = {0,0};
		static float far_end_voice_max[2] = {0,0};//DTD 
#endif

#ifdef NLMS
		static float squart_far_end[2] ={0,0};
		static int start_cnt[2] = {0,0};
#endif
#endif

void give_to_echo_cancel(unsigned char* down_Buff,unsigned char* up_Buff, int data_size, unsigned char* result)
{
	 		 		
		//0-decode, pcm-to-linear
		for(int i=0; i<16; i++){
				down_Buff_linear[1 +i*12] = decode(down_Buff[1+i*12]); //decode pcm2Linear
				up_Buff_linear[1+i*12]    = decode(  up_Buff[1+i*12]); //decode			
				down_Buff_linear[2 +i*12] = decode(down_Buff[2+i*12]); //decode pcm2Linear
				up_Buff_linear[2+i*12]    = decode(  up_Buff[2+i*12]); //decode
		}
				
    const float u = 0.000000000005; 			// 0.0000000005;

		short* far_end_addr 	= (short*) down_Buff_linear;
		short* near_end_addr 	= (short*) up_Buff_linear;	 
				
		short far_end_voice = 0;
		short near_end_voice = 0;
		float far_end_voice_f = 0.0f;
		float near_end_voice_f = 0.0f;

		float e = 0.0f;	//err
		float y = 0.0f;	//sum-result 
		
		far_end_addr 	= (short*) down_Buff_linear;
		near_end_addr = (short*) up_Buff_linear;	
		
		far_end_voice 	 = 0;
		near_end_voice 	 = 0;
		far_end_voice_f  = 0.0f;
		near_end_voice_f = 0.0f;

		e = 0.0f;	//err
		y = 0.0f;	//sum-result 
						
		static float far_end_voice_array[2][ORDER] = {1};
		static float w[2][ORDER] = {0};
#ifdef DTD		 
		static int max_window_cnt[2] = {0,0};
		static float far_end_voice_max[2] = {0,0};//DTD 
#endif

#ifdef NLMS	 
		static float squart_far_end[2] ={0,0};
		static int start_cnt[2] = {0,0};
#endif

#ifdef DEBUG_ECHO
		static int frame_cnt = 0;		
		if(frame_cnt < 1000)
			frame_cnt++;
		else
			frame_cnt = 0;
#endif
		 						
		for (int ch=0; ch<2; ++ch)//12-channel	
		{	
				for (int i=0; i<data_size; ++i)// data_size 
				{				
					y = 0;
					far_end_voice 	= *(far_end_addr  + i*12 + ch + 1);	//channel-number:0
					near_end_voice 	= *(near_end_addr + i*12 + ch + 1);
					far_end_voice_f  = (float)far_end_voice;
					near_end_voice_f = (float)near_end_voice;
					
					#ifdef DTD
					/*******************DTD start************************/
					if ( fabsf(far_end_voice_f) > far_end_voice_max[ch] )//DTD
					{
							far_end_voice_max[ch] = fabsf(far_end_voice_f);
							max_window_cnt[ch] = 0;
					}
					else
					{
							if (max_window_cnt[ch] < ORDER){							
									max_window_cnt[ch]++;
							}
							else{							
									max_window_cnt[ch] = 0;
									far_end_voice_max[ch] = fabsf(far_end_voice_f);
							}
					}
					/*******************DTD end************************/
					#endif

					#ifdef NLMS
					if (start_cnt[ch] < ORDER+1){				
							start_cnt[ch]++;
					}
					if (start_cnt[ch] < ORDER+1){
							squart_far_end[ch] = squart_far_end[ch] + far_end_voice_f*far_end_voice_f;
					}else{					
							squart_far_end[ch] = squart_far_end[ch] - far_end_voice_array[ch][0]*far_end_voice_array[ch][0];
							squart_far_end[ch] = squart_far_end[ch] + far_end_voice_f*far_end_voice_f;
					}
					#endif

					//self-adaption-filter			
					for (int j=0; j<ORDER-1; ++j){				
							far_end_voice_array[ch][j] = far_end_voice_array[ch][j+1];		//loop-shitf
							y = y + far_end_voice_array[ch][j] * w[ch][ORDER-1-j];				//conv-func				  
					}				
					far_end_voice_array[ch][ORDER-1] = far_end_voice_f;
					y = y + w[ch][0]*far_end_voice_array[ch][ORDER-1];					
					e = near_end_voice_f - y;
					#ifdef DTD
						if(fabs(near_end_voice_f) < far_end_voice_max[ch]*0.5)
					#else
						if ( 1 )  //update-filter-paramter
					#endif			
					{
							for (int k = 0; k < ORDER; ++k){							
									#ifdef NLMS
											w[ch][k] = w[ch][k] + 0.4*e*far_end_voice_array[ch][ORDER-1-k]/(0.0001 + squart_far_end[ch]);//[0, 2]: 0.4  0.3 0.2						
									#else				
											w[ch][k] = w[ch][k] + u*e*far_end_voice_array[ch][ORDER-1-k];
									#endif	
							}
							
							#ifdef DEBUG_ECHO
							if (far_end_voice_f > 500.0f && (frame_cnt == 1000)){
								for(int i=0; i<ORDER; i++){
									 if(w[ch][i] > 0.05f){
												printf("%f ", w[ch][i]);
										 }		
									 }
								printf("\n");
							 }
							#endif
					}					
					processed_addr[i] = (short)(e);						
					// decode, 
					result[ch+1+i*12] = encode(processed_addr[i]);	
							
			 }		
		 }
}

/*******************************************************************
		desc: not-completed
		

*******************************************************************/
void echo_cancel_func(unsigned char *far_end_addr,  unsigned char *near_end_addr, 														 
															      int voice_len,  unsigned char *e_char)
{
			
			const float u = 0.0001f; 	 //0.000002f
			unsigned short far_end_voice_uchar  = 0;
			unsigned short near_end_voice_uchar = 0;	
			unsigned char far_end_voice_delay_unchar = 0;
			float far_end_voice_f  = 0.0f;
			float near_end_voice_f = 0.0f;
			float far_end_voice_delay_f = 0.0f;
			float err = 0.0f; 
			float y = 0.0f; 
			float far_end_voice_max = 0.0f; 
			static int turn_cnt = 0;
			static float far_end_voice_array[ORDER] = {1.0f};
			static float filter_coefficient[ORDER]  = {0.0f};
			//short res_pcm2Linear[2]={0};			
			//total-12-channels, so-need-12-loops
			#if 1
			for(int loop=0; loop < voice_len; loop+=2) //voice_len = RxTx_SIZE/40
			{			 
				  for (int i=0; i<12; ++i) //12
					{						    
						    far_end_voice_max = 0;														    					   																					
								far_end_voice_delay_unchar = far_end_addr[i];
								near_end_voice_uchar 			 = near_end_addr[i];						
								//far_end_voice_uchar 	= pcm2Linear(far_end_addr[i+loop*12]); 	
								//near_end_voice_uchar	= pcm2Linear(near_end_addr[i+loop*12]);		
																													
								far_end_voice_f  = (float)far_end_voice_uchar  - 128;
								near_end_voice_f = (float)near_end_voice_uchar - 128;						
								//far_end_voice_delay_f = (float)far_end_voice_delay_unchar - 128;
								
								//process the near_end_voice_f
								//near_end_voice_f = near_end_voice_f + far_end_voice_delay_f*0.45;
								
								y = 0; 
								for (int j = 0; j < ORDER-1; ++j){													
										far_end_voice_array[j] = far_end_voice_array[j+1];    
										y += far_end_voice_array[j] * filter_coefficient[ORDER-1-j];       
								}
								
								far_end_voice_array[ORDER-1] = far_end_voice_f;
								y += filter_coefficient[0]*far_end_voice_array[ORDER-1];								 
								err = near_end_voice_f - y;							  					
								if(1)
								{
										for (int k=0; k<ORDER; ++k){						
												filter_coefficient[k] += u*err*far_end_voice_array[ORDER-1-k];
										}
								}
								else
								{			 
										//printf("detect silent pieces %d--%d\n", turn_cnt, i);
								}		 					
								e_char[i+loop*12] = (unsigned char)(err + 128);		 
																
					}	//end-for			
			}	
			#endif	
}
