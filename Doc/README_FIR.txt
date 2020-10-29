/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2012a\extern\include\tmwtypes.h 
 */
/*
 * Warning - Filter coefficients were truncated to fit specified data type.  
 *   The resulting response may not match generated theoretical response.
 *   Use the Filter Design & Analysis Tool to design accurate
 *   int16 filter coefficients.
 */
const int BL = 6;
const int16_T B[6] = {
	  3519,   5123,   6610,   6610,   5123,   3519
};


    #if FIR
    	/**
    	* pcmData : input PCM array, int16 stero PCM
    	* pcmData_fir : output PCM array, int16 stero PCM
    	*/
    	printf("===fir===\n");
    	uint32_t n,m;
    	int32_t yn;				
    	int16_t* pcmdata_p = (int16_t*)pcmData;
    	//--------------left PCM channel-------------
    	for(n=0;n< sizeof(pcmData) /2;n++)
    	{
    		yn=0;
    		for(m=0;(m<BL)&&(m<n);m++)
    			yn+=B[m]*pcmdata_p[2*(n-m)];
    		pcmData_fir[2*n]= yn>>(10+5);
    	}
    	//--------------right PCMchannel-------------
    	for(n=0;n< sizeof(pcmData) /2;n++)
    	{
    		yn=0;
    		for(m=0;(m<BL)&&(m<n);m++)
    			yn+=B[m]*pcmdata_p[2*(n-m)+1];
    		pcmData_fir[2*n+1]= yn>>(10+5);
    	}
    #endif	