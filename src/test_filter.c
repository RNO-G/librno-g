//the goal of this is to compute variables of the event to try to filter to a lower data rate
//trigger rate ~20Hz -> collection rate ~1Hz...
//so by using the 20Hz, it takes 1/20s per event now. if these calculations take another 1/20s
//we would then be at a trigger rate of 10Hz... better
#include <time.h> 
#include <sys/types.h> 
#include <stdio.h>
#include <unistd.h> 
#include <sys/file.h> 
#include <stdlib.h> 
#include <string.h> 
#include <fcntl.h> 
#include <errno.h> 
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <assert.h> 
#include <errno.h> 
#include <math.h> 
#include <inttypes.h>
#include <stdint.h>
#include <complex.h>
#include <fftw3.h>
#include "test_filter.h"


#if defined(__arm__)
#include <arm_neon.h>   
#endif


//#define NCHAN 24
//#define NSAMP 1024


int real_time_calc_rms(real_time_t * rt)
{
    for(uint16_t i=0;i<4;i++)
    {
        float temp_rms=0;
        rt->rms[i]=0;
        //uint8_t ichan;
        uint16_t isam;
        //printf("calculating rms\n");
        //sum sqaures
        //float temp_val=0;
        for(isam=0;isam<2048;isam++)
        {
            rt->squared_waveforms[i][isam]=(float)rt->event_waveform[i][isam]*(float)rt->event_waveform[i][isam];
            //printf("rms %f\n",temp_rms);
            //printf("wave %i\n",waveform[isam]);
            //printf("val %i\n",waveform[isam]);
            //temp_val=rt->event_waveform[i][isam];
            temp_rms+=rt->squared_waveforms[i][isam];
            //fast and dirty power rms[ichan]+=waveforms[ichan][isam]<<2;
        }
        //printf("went through waveform\n");
        //mean square
        rt->rms[i]=temp_rms/2048.;//integer division ok
        //root
        rt->rms[i]=sqrt(rt->rms[i]);
    }
    return 0;
}

int real_time_calc_snr(real_time_t * rt)
{
    int16_t max_adc=0;
    int16_t min_adc=0;

    for(uint16_t i=0;i<4;i++)
    {
        max_adc=0;
        min_adc=0;
        for(uint16_t j=0;j<2048;j++)
        {
            if(rt->event_waveform[i][j]>max_adc) max_adc=rt->event_waveform[i][j];
            if(rt->event_waveform[i][j]<min_adc) min_adc=rt->event_waveform[i][j];
        }
        rt->snr[i] = (float)(max_adc-min_adc)/(2*(rt->rms[i]));
    }

    return 0;
}

int real_time_load_template(real_time_t * rt)
{
    FILE * t_wave_file;
    t_wave_file=fopen("~/librno-g/helper/template.dat","rb");
    uint16_t ret=0;
    ret+=fread(rt->template_waveform,2,2048,t_wave_file);
    //printf("%i ret\n",ret);
    fclose(t_wave_file);
    if(ret!=2048*2) return -1;
    return 0;
}

int real_time_load_fft(real_time_t * rt)
{
    //allocate space needed for fft buffers, targets, and waveform ffts
    rt->in_fft=fftwf_alloc_complex(2048);
    rt->out_fft=fftwf_alloc_complex(2048);
    rt->fft_buffer1=fftwf_alloc_complex(2048);
    rt->fft_buffer2=fftwf_alloc_complex(2048);
    rt->template_fft=fftwf_alloc_complex(2048);

    rt->event_fft[0]=fftwf_alloc_complex(2048);
    rt->event_fft[1]=fftwf_alloc_complex(2048);
    rt->event_fft[2]=fftwf_alloc_complex(2048);
    rt->event_fft[3]=fftwf_alloc_complex(2048);


    //load/calc the plan
    fftwf_import_wisdom_from_filename("~/librno-g/helper/imp_wisdom.dat"); 
    rt->forward = fftwf_plan_dft_1d(2048, rt->in_fft, rt->out_fft, FFTW_FORWARD, FFTW_MEASURE);
    rt-> backward = fftwf_plan_dft_1d(2048, rt->out_fft, rt->in_fft, FFTW_BACKWARD, FFTW_MEASURE);
    fftwf_export_wisdom_to_filename("~/librno-g/helper/imp_wisdom.dat");
  
    return 0;
}

int real_time_free_fft(real_time_t * rt)
{
    fftwf_free(rt->in_fft);
    fftwf_free(rt->out_fft);
    fftwf_free(rt->fft_buffer1);
    fftwf_free(rt->fft_buffer2);
    fftwf_free(rt->template_fft);
    fftwf_free(rt->event_fft[0]);
    fftwf_free(rt->event_fft[1]);
    fftwf_free(rt->event_fft[2]);
    fftwf_free(rt->event_fft[3]);

    fftwf_destroy_plan(rt->forward);
    fftwf_destroy_plan(rt->backward);

    return 0;
}

int real_time_calc_event_fft(real_time_t * rt)
{

    //load waveforms into targets, do ffts, and offload
    for(uint16_t channel=0;channel<4;channel++)
    {
        for(uint16_t sam=0;sam<2048;sam++)
        {
            rt->in_fft[sam]=rt->event_waveform[channel][sam];
        }
        fftwf_execute(rt->forward); //no normalization...
        memcpy(rt->event_fft[channel],rt->out_fft,sizeof(rt->out_fft[0])*2048);//send to event fft 
    }    
    return 0;
}
int real_time_calc_template_fft(real_time_t * rt)
{
    for(uint16_t sam=0;sam<2048;sam++)
    {
        rt->in_fft[sam]=rt->template_waveform[sam];
    }
    fftwf_execute(rt->forward); //no normalization...
    for(uint16_t sam=0;sam<2048;sam++)
    {
        rt->template_fft[sam]=rt->out_fft[sam];
    }
    //memcpy(rt->template_fft,rt->out_fft,sizeof(rt->out_fft[0])*2048);//send to template fft
    return 0;

}

int real_time_calc_impulsivity(real_time_t * rt)
{
    float hilbert_integral=0;
    uint16_t imp_range=50;
    uint16_t index_max=0;
    float max_env=0;
    float cdf[2048]={0};

    for(uint8_t channel=0;channel<4;channel++)
    {
        //memcpy(rt->out_fft,rt->event_fft[channel],sizeof(rt->event_fft[channel][0])*2048);
        
        for(uint16_t l =0 ;l<2048;l++)
        {
            rt->out_fft[l]=rt->event_fft[channel][l];
            //apply the fft->analytic signal filter
            if(l==0) continue; //rt->fft_out[l]=rt->fft_out[l];
            else if(l>0&&l<(2048/2-1)) rt->out_fft[l]=2*rt->out_fft[l];
            else if(l==2048/2-1) continue; //rt->fft_out[l]=rt->fft_out[l];
            else rt->out_fft[l]=0;
        }

        fftwf_execute(rt->backward);
        for(uint16_t i=0;i<2048;i++)
        {
            rt->hilbert_env[channel][i]=cabs(rt->in_fft[i])/2048.; //2048 for fft normalization
            if(rt->hilbert_env[channel][i]>max_env)
            {
                max_env=rt->hilbert_env[channel][i];
                index_max=i;
            }
            if(i==0)cdf[i]=rt->hilbert_env[channel][i];
            else cdf[i]=cdf[i-1]+rt->hilbert_env[channel][i];
        }
        
        hilbert_integral=cdf[2047];
        //printf("integral %f\n",hilbert_integral);
        
        if(index_max<imp_range)
        {
            rt->impulsivity[channel]=(cdf[index_max+imp_range]-cdf[index_max])/hilbert_integral;
        }
        else if(index_max>(2047-imp_range))
        {
            rt->impulsivity[channel]=(cdf[2047]-cdf[index_max-imp_range])/hilbert_integral;        
        }
        else
        {
            rt->impulsivity[channel]=((cdf[index_max+imp_range]-cdf[index_max-imp_range]))/hilbert_integral;
        }
        //printf("impul: %f\n",rt->impulsivity[channel]);
    }

    return 0;
 
}
int real_time_calc_template_match(real_time_t * rt)
{
    float normalization;

    float abs_corr[2048];
    int i_max=0;
    float temp_corr;
    for(uint8_t i=0;i<4;i++)
    {
        normalization=1;
        for(int sam=0;sam<2048;sam++)
        {
            rt->in_fft[sam]=rt->event_waveform[i][sam];//*event[0][sam];//fill array with first waveform
        }
        fftwf_execute(rt->forward); //no normalization... gets the fft of the data and fills it to out_data bc useful
        //printf("size of comp float %i\n",sizeof(in_data[0]));
        //memcpy(in_data2,out_data,sizeof(out_data[0])*2048);//send to extra space
        
        
        for(int sam=0;sam<2048;sam++)
        {
            rt->out_fft[sam]=rt->out_fft[sam]*conj(rt->out_fft[sam]);
        }
        fftwf_execute(rt->backward);
        normalization=cabs(rt->in_fft[0]);
        temp_corr=0;
        rt->template_corr[i]=0;
        memset(abs_corr,0,2048*4);

        for(uint16_t sam=0;sam<2048;sam++)        
        {
            normalization+=rt->event_waveform[i][sam];
            rt->out_fft[sam]=rt->event_fft[i][sam]*conjf(rt->template_fft[sam]);
        }
        fftwf_execute(rt->backward);

        for(uint16_t sam=0;sam<2048;sam++)        
        {
            //printf("temp %f\n",abs_comp[l]);

            abs_corr[sam]=cabs(rt->in_fft[sam])/normalization;
            //printf("abs of iff(fft1*fft2) %f\n",cabs(comp1[l]));

            if(abs_corr[sam]>temp_corr)
            {
                //printf("in here");
                i_max=sam;
                temp_corr=abs_corr[sam]; //max correlation of the template
            }
        }
        rt->template_corr[i]=temp_corr;
    }

    return 0;
}
int real_time_calc_phase_delays(real_time_t * rt)
{
    memset(rt->phased_sum,0,2048*4);
    float max_cross=0;
    int16_t i_max=0; 
    float cross[2048]={0.};
    uint8_t loudest_channel=0;
    float max_snr=0;
    for(uint8_t i=0;i<4;i++)
    {
        //printf("snr %f\n",rt->snr[i]);
        if((rt->snr[i])>max_snr)
        {
            loudest_channel=i;
            max_snr=rt->snr[i];
        }
    }

    //printf("loudest channel %i\n",loudest_channel);

    rt->phased_delays[loudest_channel]=0; //base the rest of the calc on this one
    //printf("loudest channel %i\n",loudest_channel);
    
    //memcpy(rt->fft_buffer1,rt->event_fft[loudest_channel],sizeof(rt->fft_buffer1[0]*2048));
    for(uint8_t channel=0;channel<4;channel++)
    {
        if(channel==loudest_channel) {rt->phased_delays[channel]=0; continue;}
        
        for(uint16_t l =0 ;l<2048;l++)
        {
            rt->out_fft[l]=rt->event_fft[channel][l]*conj(rt->event_fft[loudest_channel][l]);
        }
    
        //reverse fft 
        fftwf_execute(rt->backward);  //again no normalization.. so to make sense of the same waveform 1/n has to be applied to the output on x
        max_cross=0;
        for(int16_t i =0;i<2048;i++)
        {
            //I want to weight the shifts so no shift -> physical limit is passed and outside of that is likely ignored
            //printf("real %f, imag %f\n",creal(rt->in_fft[i]),cimag(rt->in_fft[i]));

            cross[i]=cabs(rt->in_fft[i])/(2048);
            //printf("cross cor %f\n",cross[i]);
            //printf("index %i abs is %f\n",i,cross[i]);
            //printf("cross[i] %f\n",cross[i]);
            if(cross[i]>max_cross)
            {
                max_cross=cross[i];
                i_max=i;
            }
        
        }
        //printf("i max %i and max cross cor %f\n",i_max,max_cross);
        //printf("max index %i\n",i_max);
        //*best_corr=0;
        //for(int i =0;i<50;i++)
        //{
            //printf("%f\n",cross[i_max-(5-i)]);
        //    *best_corr+=cross[i_max-(25-i)]; //sum a range around the peak since power is slightly distributed
        //}
        //printf("fft best shift is %i at a cross corr of %f\n",i_max,max_cross);
        //*best_corr=max_cross; //pick only the peak
        //printf("sum corr = %f\n",*best_corr);
        if(i_max>1024)rt->phased_delays[channel]=i_max-2048;
        else rt->phased_delays[channel]=i_max;
        //printf("rt->phased_delays[channel]=%i\n",rt->phased_delays[channel]);

    }
    
    return 0;
}
int real_time_calc_phased_integral(real_time_t * rt)
{
    float integrated_power_sum=0;
    float pow_int=0;
    for(uint8_t channel=0;channel<4;channel++)
    {
        //printf("channel diff %i\n",rt->phased_delays[channel]);
        int16_t index=0;
        for(uint16_t sam=0;sam<2048;sam++)
        {
            index=sam+rt->phased_delays[channel];
            if(index>2047) index=index-2048;
            if(index<0) index=index+2048;
            if(index<0 || index>2047)
            {
                printf("oops %i",index);
                return -1;
            }
            rt->phased_sum[sam]+=rt->squared_waveforms[channel][index];
            //printf("squared wave[sam] %f",rt->squared_waveforms[channel][index]);
            //printf("phased sum[sam] %f",rt->phased_sum[index]);
        }
    }


    //okay now integrate around the peak I guess
    uint16_t peak_i=0;
    float max_phased=0;
    float full=0;


    for(uint16_t l=0;l<2048;l++)
    {
        //printf("%f ",phased_power_sum[l]);
        full+=rt->phased_sum[l];

        if(max_phased<rt->phased_sum[l])
        {   
            //printf("here");
            peak_i=l;
            max_phased=rt->phased_sum[l];
        }
        
    }
    //printf("peak i %i, max phased % f\n",peak_i,max_phased);
    int16_t index=0;
    for(uint16_t sam=0;sam<50;sam++)
    {
        index=peak_i+sam-25;
        if(index<0)index=index+2048;
        if(index>2047)index=index-2048;
        if(index>2047 || index <0)
        {
            printf("oops %i\n",index);
            return -1;
        }
        integrated_power_sum+=rt->phased_sum[index];
    }
    //printf("full is %f\n",full);
    rt->phased_integral=integrated_power_sum/full;
    //printf("integrated power sum %f\n",integrated_power_sum);
    //printf("normalized integrated power sum %f\n",rt->phased_integral);

    return 0;
}

int real_time_set_thresh(real_time_t * rt)
{
    //simple 'method' to set thrshold values. easy look up.
    rt->snr_thresh=4.0;
    rt->impulsivity_thresh=0.05;
    rt->phased_thresh=0.0415;
    rt->template_thresh=0.08;
    rt->shift_thresh=150;

    return 0;
}

int real_time_set_transform(real_time_t * rt)
{
    float use_xbars[9]={0.08393662,0.08136263,0.07885022,0.07537819,0.09585343,0.10497387,0.10267792,0.14253856,0.12747116};
    float use_scaling[9]={3.4360976,-7.4730453,-1.4640616,0.80900156,-15.504141,3.5600338,-6.3156667,-3.9813983,-2.7120492};

    for(uint8_t i=0;i<9;i++)
    {
        rt->xbars[i]=use_xbars[i];
        rt->scaling[i]=use_scaling[i];
    }
    //float xbars[9];
    //float scaling[9];
    //values generated using impulsivity, template match, and phased sum impulsivity in sklearn linear descriminant. names are xbar and scale
    //rt->xbars={0.08393662,0.08136263,0.07885022,0.07537819,0.09585343,0.10497387,0.10267792,0.14253856,0.12747116};
    //rt->scaling={3.4360976,-7.4730453,-1.4640616,0.80900156,-15.504141,3.5600338,-6.3156667,-3.9813983,-2.7120492};
}   

int real_time_calc_desc(real_time_t * rt)
{
    rt->lin_desc=0;

    rt->lin_desc+=(rt->impulsivity[0]-rt->xbars[0])*rt->scaling[0]; 
    rt->lin_desc+=(rt->impulsivity[1]-rt->xbars[1])*rt->scaling[1]; 
    rt->lin_desc+=(rt->impulsivity[2]-rt->xbars[2])*rt->scaling[2]; 
    rt->lin_desc+=(rt->impulsivity[3]-rt->xbars[3])*rt->scaling[3]; 

    rt->lin_desc+=(rt->template_corr[0]-rt->xbars[4])*rt->scaling[4];
    rt->lin_desc+=(rt->template_corr[1]-rt->xbars[5])*rt->scaling[5];
    rt->lin_desc+=(rt->template_corr[2]-rt->xbars[6])*rt->scaling[6];
    rt->lin_desc+=(rt->template_corr[3]-rt->xbars[7])*rt->scaling[7];


    

    //rt->
    //imp[0-3],temp[0-3],phased sum
    //for(uint8_t i =0;i<4;i++)
    //{
    //    rt->lin_desc+=(rt->impulsivity[i]-rt->xbars[i])*rt->scaling[i]; 
    //    rt->lin_desc+=(rt->template_corr[i]-rt->xbars[i+4])*rt->scaling[i+4];
    //}
    rt->lin_desc+=(rt->phased_integral-rt->xbars[8])*rt->scaling[8];
}

int real_time_calc_priority(real_time_t * rt)
{
    //here's where we do something with the calculated values

    //if(event[channel]>threshold)
    //add one
    //...

}

int sort_buffered_events(float * lin_events,uint8_t * sorted_places, int n_events,int index_of_best)
{
    uint8_t n_above[n_events];
    uint8_t n_below[n_events];
    for(uint8_t i=0;i<n_events;i++)
    {
        for(uint8_t j=0;j<n_events;j++)
        {
            if(i==j)continue;
            if(lin_events[i]>lin_events[j])
            {
                n_above[i]+=1;
            }
            if(lin_events[i]<lin_events[j])
            {
                n_below[i]+=1;
            }
        }
        sorted_places[i]=n_above[i];
    }
    return 0;
}

int pick_buffered_events(float * lin_events,uint8_t * sorted_places, int n_events,float threshold)
{
    uint32_t keep_these=0;
    for(uint8_t i=0;i<n_events;i++)
    {
        if(lin_events[i]<threshold) keep_these &=1<<i;
    }
    uint8_t in_best=0;
    sort_buffered_events(lin_events,sorted_places,n_events,in_best);
    keep_these&=1<<sorted_places[in_best];
    return 0;
}
//static uint16_t fake_data[NCHAN][NSAMP]; 
//static uint16_t fake_data[NSAMP]; 
int test_impulsivity();
int test_phased_power_sum()
{
    printf("starting phased power sum test\n");
    int16_t event[4][2048];
    float square_event[4][2048];
    int NSAMP=2048;
    
    fftwf_complex * in_data = fftwf_alloc_complex(NSAMP); //input data
    fftwf_complex * in_data2 = fftwf_alloc_complex(NSAMP); //input data buffer
    fftwf_complex * out_data = fftwf_alloc_complex(NSAMP); //output data

    
    fftwf_import_wisdom_from_filename("~/librno-g/helper/imp_wisdom.dat"); 
    fftwf_plan forward = fftwf_plan_dft_1d(NSAMP, in_data, out_data, FFTW_FORWARD, FFTW_MEASURE);
    fftwf_plan backward = fftwf_plan_dft_1d(NSAMP, out_data, in_data, FFTW_BACKWARD, FFTW_MEASURE);
    fftwf_export_wisdom_to_filename("~/librno-g/helper/imp_wisdom.dat");
    


    FILE *sim_file,*data_file;
    sim_file=fopen("rnog_share/filtering/processed/c_waves.dat","rb"); //felix's simulated data, 3092 events
    data_file=fopen("rnog_share/station23_processed/c_data.dat","rb"); //all event types 7375 events

    //forced_file=fopen("rnog_share/station23_processed/c_forced.dat","rb"); //forced triggers, 665 events
    //lt_file=fopen("rnog_share/station23_processed/c_lt.dat","rb"); //lt triggers, 3620 events
    //rad_file=fopen("rnog_share/station23_processed/c_rad.dat","rb"); //rad triggers, 3090 events
    
    //int rad_events=3090;
    //int forced_events=665;
    int sim_events=3092;
    int data_events=7375;

    //int lt_events=3620;

    FILE *sim_data;
    sim_data=fopen("data/phased_sim.dat","wb");

    FILE *data_data;
    data_data=fopen("data/phased_data.dat","wb");

    int ret=0;
    float rms[4];
    float snr[4];
    int loudest=0;
    int16_t shift[4];
    float peak_cross[4];
    float phased_power_sum[2048]={0.};
    float integrated_power_sum=0;

    int n_events=sim_events+data_events;
    for(int i =0;i<n_events;i++)
    {
        integrated_power_sum=0;
        if(i%100==0)printf("%i\n",i);
        ret=0;
        //load in phased array waveforms
        if(i<sim_events)
        {
            ret+=fread(event[0],2,2048,sim_file);
            ret+=fread(event[1],2,2048,sim_file);
            ret+=fread(event[2],2,2048,sim_file);
            ret+=fread(event[3],2,2048,sim_file);
        }
        else
        {
            ret+=fread(event[0],2,2048,data_file);
            ret+=fread(event[1],2,2048,data_file);
            ret+=fread(event[2],2,2048,data_file);
            ret+=fread(event[3],2,2048,data_file);
        }
    
        
 


        //ret+=fread(event,2,2048*4,py_file); this might read in the event at one time... maybe quicker?

        //get the snr's
        for(int j=0;j<4;j++)
        {
            float_rms(event[j],&rms[j]);
            calc_snr(event[j],&rms[j],&snr[j]);
            if(j>0)
            {
                if(snr[j]>snr[j-1]) loudest=j;
            }
            for(int k=0;k<2048;k++)
            {
                square_event[j][k]=(float)event[j][k]*(float)event[j][k];//divide by 2048 so it's not so big
            }
        }
        //printf("%i\n",loudest);
        
        //have loudest waveform so we should reference this to the others
        for(int sam=0;sam<2048;sam++)
        {
            in_data[sam]=event[loudest][sam];//*event[0][sam];//fill array with first waveform
            phased_power_sum[sam]=square_event[loudest][sam];
            //phased_power_sum[sam]=event[loudest][sam]; //to reduce noise?

            //printf("%f\n",phased_power_sum[sam]);
        }
        fftwf_execute(forward); //no normalization... gets the fft of the data and fills it to out_data bc useful
        //printf("size of comp float %i\n",sizeof(in_data[0]));
        memcpy(in_data2,out_data,sizeof(out_data[0])*2048);//send to extra space
        for(int j=0;j<4;j++)
        {
            
            if(j==loudest)continue;
            for(int sam=0;sam<2048;sam++)
            {   
                in_data[sam]=event[j][sam];//*event[1][sam]; //fill array with second waveform
            }
            fftwf_execute(forward); //no normalization... gets the fft of the data and fills it to out_data bc useful
            get_time_delay_from_fft(in_data, in_data2, out_data, backward, &shift[j], &peak_cross[j]);
            //printf("shift is %i\n",shift);
            
            //shift neg values
            if(shift[j]>1024)shift[j]=shift[j]-2048;
            //printf("shift is %i\n",shift[j]);
            int index=0;
            for(int sam =0;sam<2048;sam++)
            {   
                index=sam+shift[j];
                if(index>2047) index=index-2048;
                if(index<0) index=index+2048;
                if(index<0 || index>2047)
                {
                    printf("oops %i",index);
                    return -1;
                }
                //printf("index %i ",index);
                //printf("%f ",square_event[j][index]);

                phased_power_sum[sam]+=square_event[j][index];
                //phased_power_sum[sam]+=event[j][index]; //to reduce noise?
            }

        }
        //okay now integrate around the peak I guess
        int peak_i=0;
        float max_phased=0;
        for(int l=0;l<2048;l++)
        {
            //printf("%f ",phased_power_sum[l]);
            if(max_phased<phased_power_sum[l])
            {   
                //printf("here");
                peak_i=l;
                max_phased=phased_power_sum[l];
            }
            
        }
        //printf("peak i %i, max phased % f\n",peak_i,max_phased);
        int go_index=0;
        for(int ran=0;ran<50;ran++)
        {
            go_index=peak_i+ran-25;
            if(go_index<0)go_index=go_index+2048;
            if(go_index>2047)go_index=go_index-2048;
            if(go_index>2047 || go_index <0)
            {
                printf("oops %i\n",go_index);
                return -1;
            }
            integrated_power_sum+=phased_power_sum[go_index];
        }
        float full=0;

        for(int samples=0;samples<2048;samples++)
        {
            full+=phased_power_sum[samples];
        }
        //printf("integrated power sum %f\n",integrated_power_sum);
        //printf("normalized integrated power sum %f\n",integrated_power_sum/full);
        float phased_rms=0;
        float phased_snr=0;
        //float_float_rms(phased_power_sum,&phased_rms);
        //calc_float_snr(phased_power_sum,&phased_rms,&phased_snr);
        //printf("phased rms %f and phased snr %f\n",phased_rms,phased_snr);
        float norm_phase=integrated_power_sum/full;
        if(norm_phase>0.9) printf("phased int %f\n",norm_phase);
        if(i<sim_events)
        {
            fwrite(&norm_phase,4,1,sim_data); 
        }
        else //if(i<sim_events)
        {
            fwrite(&norm_phase,4,1,data_data); 
        }
    }

    //fwrite(phased_power_sum,4,2048,plot_data);4
    fclose(data_data);
    fclose(sim_data);

    fclose(sim_file);
    fclose(data_file);
    return 0;


}

int float_float_rms(float * waveform, float * rms)
{
    float temp_rms=0;
    *rms=0;
    //uint8_t ichan;
    uint16_t isam;
    //printf("calculating rms\n");
    //sum sqaures
    float temp_val=0;
    for(isam=0;isam<2048;isam++)
    {
        //printf("rms %f\n",temp_rms);
        //printf("wave %i\n",waveform[isam]);
        //printf("val %i\n",waveform[isam]);
        temp_val=waveform[isam];
        temp_rms+=temp_val*temp_val;
        //fast and dirty power rms[ichan]+=waveforms[ichan][isam]<<2;
    }
    //printf("went through waveform\n");
    //mean square
    *rms=temp_rms/2048.;//integer division ok
    //root
    *rms=sqrt(*rms);
    
    //print result
    //printf("calculated rms as %f\n",*rms);
    return 0;
}
int calc_float_snr(float * waveform,float * rms,float * snr)
{
    uint16_t i;
    float max_adc=0;
    float min_adc=0;

    for(i=0;i<2048;i++)
    {
        if(waveform[i]>max_adc) max_adc=waveform[i];
        if(waveform[i]<min_adc) min_adc=waveform[i];
    }

    *snr = (max_adc-min_adc)/(2*(*rms));
    return 0;
}



int see_perf_and_bad_snr()
{
    int16_t flat_data[2048]={0}; 
    int16_t perf_delta[2048]={0};

    for(int i =0;i<2048;i++)
    {
        perf_delta[i]=0;
        if(i==1024) perf_delta[i]=0xfff;

        flat_data[i]=1;
    }



    float perf_rms=0;
    float perf_snr=0;
    float_rms(perf_delta,&perf_rms);
    calc_snr(perf_delta,&perf_rms,&perf_snr);

    float flat_rms=0;
    float flat_snr=0;
    float_rms(flat_data,&flat_rms);
    calc_snr(flat_data,&flat_rms,&flat_snr);

    printf("perfect delta has rms of %f and snr of %f\n",perf_rms,perf_snr);
    printf("flat data has rms of %f and snr of %f\n",flat_rms,flat_snr);

    return 0;

}

int benchmark_vars()
{
    /*
    
    //make dummy data
    
    int16_t fake_data[2048]={0}; 
    int16_t perf_delta[2048]={0};

    float snr_dist[100]={0.};
    float imp_dist[100]={0.};
    float power_sum_dist[100]={0.};
    

    perf_delta[100]=400;    
    
    int16_t perf_noise[2048]={0};

    //memset(perf_noise,4,2048*sizeof(perf_noise[0])); doesnt actually set to the right int

    for(int i =0;i<2048;i++)
    {
        if(i%3==0)perf_noise[i]+=2;
        if(i%2==0)perf_noise[i]+=-3;
        if(i%5==0)perf_noise[i]+=4;
        //else perf_noise[i]=-1;
        //if(i==1000)perf_noise[i]=;
        //printf("%i ",perf_noise[i]);
    }

    printf("start here\n");
    //FILE * frandom = fopen("/dev/urandom","r"); 
    //fread(fake_data, 1024*2, 1, frandom); 
    //fclose(frandom);

    //for(int i =0;i<2048;i++)
    //{
        //fake_data[i]=i%3;
    //}
    memcpy(fake_data,perf_delta,2*2048);
    //memcpy(fake_data,perf_noise,2*2048);

    FILE * fake_file;
    fake_file=fopen("fake_file.txt","w");
    for(int k = 0;k<2048;k++)
    {
        fprintf(fake_file,"%i\n",fake_data[k]);
    }
    fclose(fake_file);
    printf("made dummy data \n\n");


    //maybe read in real waveform?
    int16_t real_data[2048]={0};

    printf("reading in real data\n");

    //File * waveform_file = fopen("waveform.dat","r");
    //fread(real_data,2048,1,waveform_file);
    //fclose(waveform_file);
    int16_t real_noise[2048]={0};

    FILE * fp;
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    int count=0;

    fp = fopen("single_real_wave.txt", "r");
    if (fp == NULL) exit(EXIT_FAILURE);

    while ((read = getline(&line, &len, fp)) != -1) 
    {
        //printf("Retrieved line of length %zu:\n", read);
        //printf("%s", line);
        real_data[count]=atoi(line);
        count++;
    }
    fclose(fp);

    fp = fopen("single_real_noise.txt", "r");
    if (fp == NULL) exit(EXIT_FAILURE);
    count=0;
    while ((read = getline(&line, &len, fp)) != -1) 
    {
        //printf("Retrieved line of length %zu:\n", read);
        //printf("%s", line);
        real_noise[count]=atoi(line);
        count++;
    }
    fclose(fp);

    fp = fopen("snr_dist.txt", "r");
    if (fp == NULL) exit(EXIT_FAILURE);
    count=0;
    while ((read = getline(&line, &len, fp)) != -1) 
    {
        //printf("Retrieved line of length %zu:\n", read);
        //printf("%s", line);
        snr_dist[count]=atof(line);
        count++;
    }
    fclose(fp);

    fp = fopen("imp_dist.txt", "r");
    if (fp == NULL) exit(EXIT_FAILURE);
    count=0;
    while ((read = getline(&line, &len, fp)) != -1) 
    {
        //printf("Retrieved line of length %zu:\n", read);
        //printf("%s", line);
        imp_dist[count]=atof(line);
        count++;
    }
    fclose(fp);

    fp = fopen("power_sum_dist.txt", "r");
    if (fp == NULL) exit(EXIT_FAILURE);
    count=0;
    while ((read = getline(&line, &len, fp)) != -1) 
    {
        //printf("Retrieved line of length %zu:\n", read);
        //printf("%s", line);
        power_sum_dist[count]=atof(line);
        count++;
    }
    fclose(fp);

    memcpy(fake_data,real_noise,2*2048);


    //for(int i =0;i<2048;i++)
    //{
    //   printf("%i\n",real_data[i]);
    //}

    
      //lets mess with neon intrinsics for second
    //uint16_t huh[4]={};
    //uint16x4_t a={0xffff,0xffff,1,1};
    //uint16x4_t b={0xffff,1,2,2};
    //float first=0xffffffff;
    //float second=0xffffffff;
    //printf("%f\n",first*second);
    //uint16x4_t result={};
    //result=vadd_u16(a,b);
    //vst1_u16(huh,result);


    //for(int i =0;i<4;i++)
    //{
    //    printf("\n");
    //    printf("%i",huh[i]);
    //    printf("idk maybe %i\n",result[i]);
   // }
    //return 0;
    
  

    //set up vars and timing
    float rms=0.;
    float o_rms=0.;
    float vec_rms=0.;

    float snr=0.;
    float dt=0;

    struct timespec start;
    struct timespec end;
    

    //uint16x4_t b={0,0,0,0};
    //uint16_t huh[4]={};



    //time old_rms with mostly int math
    clock_gettime(CLOCK_MONOTONIC,&start);

    for (int i = 0; i < 1000*24; i++) 
    {
        old_rms(real_data,&o_rms);
    }

    clock_gettime(CLOCK_MONOTONIC,&end);
    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 
    printf("old rms took %f s per event\n",dt/1000);

    
    //time mostly float rms
    clock_gettime(CLOCK_MONOTONIC,&start);

    for (int i = 0; i < 1000*24; i++) 
    {
        float_rms(real_data,&rms);
    }

    clock_gettime(CLOCK_MONOTONIC,&end);
    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec);

    printf("float rms took %f s per event\n",dt/1000);

    //time neon intrinsics boosted rms
    clock_gettime(CLOCK_MONOTONIC,&start);

    //for (int i = 0; i < 1; i++) 
    for (int i = 0; i < 1000*24; i++) 
    {
        vector_rms(real_data,&vec_rms);
    }

    clock_gettime(CLOCK_MONOTONIC,&end);
    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec);

    printf("vectorized rms took %f s per event\n\n",dt/1000);



    //time mostly float snr
    clock_gettime(CLOCK_MONOTONIC,&start);

    for (int i = 0; i < 1000*24; i++) 
    {
        calc_snr(real_data,&rms,&snr);
    }

    clock_gettime(CLOCK_MONOTONIC,&end);
    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec);
    printf("old snr took took %f s per event\n\n",dt/1000);

    //time neon intrinsics boosted snr




    //
    printf("mostly int rms %f\n",o_rms);
    printf("floating rms %f\n",rms);
    printf("vectorized rms %f\n\n",vec_rms);

    printf("snr %f\n\n",snr);

    //return 0;
    //float rms[24]={0.};
    //for(count=0;count<24;count++)
    //{
    //    rms[count]=float_rms(fake_data)
    //}

    


    int NSAMP=2048;
    int ntimes = 20; 


    fftwf_complex * r_dats = fftwf_alloc_complex(NSAMP); //real time signal
    //float * r_dats = fftwf_alloc_real(NSAMP); //real time signal
    
    fftwf_complex * a_sig = fftwf_alloc_complex(NSAMP); //complex fft from real
    fftwf_complex * time_sig = fftwf_alloc_complex(NSAMP);  //complex time for analytic signal
    
    fftwf_import_wisdom_from_filename("anal_wisdom.dat"); 
    
    fftwf_plan forward = fftwf_plan_dft_1d(NSAMP, r_dats, a_sig, FFTW_FORWARD, FFTW_MEASURE);
    //fftwf_plan forward = fftwf_plan_dft_r2c_1d(NSAMP, r_dats, a_sig, FFTW_MEASURE); 
    fftwf_plan backward = fftwf_plan_dft_1d(NSAMP, a_sig, time_sig, FFTW_BACKWARD, FFTW_MEASURE);

    fftwf_export_wisdom_to_filename("anal_wisdom.dat");
    
    clock_gettime(CLOCK_MONOTONIC,&start);
    for(int i =0;i<ntimes*24;i++)
    {
        //memcpy(r_dats,real_data,2048);
        for (int j = 0; j < NSAMP; j++) 
        {
            r_dats[j] = fake_data[j]; 
            //if(i==0)printf("%f\n",r_dats[j]);
            //r_dats[j] = real_data[j]; 

        }

        fftwf_execute(forward); //there's no normalization applied. the first half of X is filled with the + freq comp. the second hald is empty in this case
        
        for(int l =0 ;l<2048;l++)
        {
            //if(creal(a_sig[l])<0.00001)
            //{
            //    printf("bad fft\n");
            //    return 0;
            //}
            //apply the fft->analytic signal filter?
            if(l==0) a_sig[l]=a_sig[l];
            else if(l>0&&l<(2048/2-1)) a_sig[l]=2*a_sig[l];
            else if(l==2048/2-1) a_sig[l]=a_sig[l];
            else a_sig[l]=0;
        }
        //memset(a_sig+1024,0,1024);
        
        //reverse fft 
        fftwf_execute(backward);  //again no normalization.. so to make sense of the same waveform 1/n has to be applied to the output on x
        if(i==0)
        {

            //FILE * for_output;
            //for_output=fopen("output_forward.txt","w");
            //for(int k = 0;k<2048;k++)
            //{
            //    fprintf(for_output,"%f+%fj\n",creal(X[k]),cimag(X[k]));
            //}
            //fclose(for_output);

            FILE * anal_output;
            anal_output=fopen("analytic_sig.txt","w");
            for(int k = 0;k<2048;k++)
            {
                //if(creal(time_sig[k])<0.0000001)
                //{
                //    printf("bad ifft\n");
                //    return 0;
                //}
                fprintf(anal_output,"%f+%fj\n",creal(time_sig[k]),cimag(time_sig[k]));
            }
            fclose(anal_output);
        }           
    }  
    clock_gettime(CLOCK_MONOTONIC,&end);

    
    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 
    printf("took %f per event to do forward and backward fft\n",dt/(ntimes));
    //printf("Processed %d \"events\" in %g (%g Hz, %g channels/second)\n", ntimes, dt, ntimes/dt, ntimes*NCHAN/dt);

    

    
    
    
    
    //fftw test
    
    //if (nargs > 1) ntimes = atoi(args[1]); 
    float * x = fftwf_alloc_real(NSAMP); 
    fftwf_complex * X = fftwf_alloc_complex(NSAMP); 


    

    fftwf_import_wisdom_from_filename("wisdom.dat"); 

    fftwf_plan fw = fftwf_plan_dft_r2c_1d(NSAMP, x, X, FFTW_MEASURE); 
    fftwf_plan bw = fftwf_plan_dft_c2r_1d(NSAMP, X, x, FFTW_MEASURE); 

    fftwf_export_wisdom_to_filename("wisdom.dat");

    clock_gettime(CLOCK_MONOTONIC,&start);

    for (int i = 0; i < ntimes*24; i++) 
    {

        for (int j = 0; j < NSAMP; j++) 
        {
            x[j] = real_data[j]; 
        }

        fftwf_execute(fw); //there's no normalization applied. the first half of X is filled with the + freq comp. the second hald is empty in this case
        
        //for(int l =0 ;l<2048;l++)
        //{
        //    if(l==0) a_sig[l]=X[l];
        //    elif(l>0&&l<2048/2-1) a_sig=2*X[l];
        //    elif(l==N/2) a_sig[l]=X[l];
        //    elif a_sig[l]=0;
        //}
        
        fftwf_execute(bw);  //again no normalization.. so to make sense of the same waveform 1/n has to be applied to the output on x
        
        if(1 && i==0)
        {

            FILE * for_output;
            for_output=fopen("output_forward.txt","w");
            for(int k = 0;k<2048;k++)
            {
                fprintf(for_output,"%f+%fj\n",creal(X[k]),cimag(X[k]));
            }
            fclose(for_output);

            FILE * rev_output;
            rev_output=fopen("output_reverse.txt","w");
            for(int k = 0;k<2048;k++)
            {
                fprintf(rev_output,"%f+%fj\n",creal(x[k]),cimag(x[k]));
            }
            fclose(rev_output);
        }

    }
    
    //crealf() and cimagf() are used with complex floats coming from complex.h
    //f is for floats

    clock_gettime(CLOCK_MONOTONIC,&end);
    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 
    printf("took %f per event to do forward and backward fft\n",dt/ntimes);
    //printf("Processed %d \"events\" in %g (%g Hz, %g channels/second)\n", ntimes, dt, ntimes/dt, ntimes*NCHAN/dt);
    
    int do_many=20;
    //impulsivity time since we have the hilbert envelope. could also do a phase matching too probably
    float hil_env[2048]={0.};
    float cdf[2048]={0.};
    float hil_int=0.;
    int imp_range=3;
    int index_of_max=0;
    float max=0.;
    float impulsivity=0.;
    clock_gettime(CLOCK_MONOTONIC,&start);
    for(int counting=0;counting<do_many*24;counting++)
    {

    
    for(int i =0;i<2048;i++)
    {
        //if(cabs(time_sig[i])<0.0000001)
        //{
        //    printf("bad abs\n");
        //    return 0;
        //}
        hil_env[i]=cabs(time_sig[i])/2048.;
        if(counting==0)
        {
            //printf("%f\n",hil_env[i]);
            //printf("%f,",creal(time_sig[i]));
            //printf("%f\n",cimag(time_sig[i]));

        }
        if(hil_env[i]>max)
        {
            max=hil_env[i];
            index_of_max=i;
        }
        if(i==0) cdf[0]=hil_env[0];
        else cdf[i]=cdf[i-1]+hil_env[i];

    }
    hil_int=cdf[2047];
    for(int i =0;i<2048;i++)
    {
        cdf[i]=cdf[i]/hil_int;
        //printf("%f\n",cdf[i]);
    }

    

    //printf("integral of hilbert env %f",hil_int);

    if(index_of_max<imp_range)
    {
        impulsivity=(cdf[index_of_max+imp_range]-cdf[index_of_max]);
        //impulsivity=2*((cdf[index_of_max+imp_range]-cdf[index_of_max])/(index_of_max+imp_range))/hil_int-1;

    }
    else if(index_of_max>2047-imp_range)
    {
        impulsivity=(cdf[2047]-cdf[index_of_max-imp_range]);
        //impulsivity=2*((cdf[2047]-cdf[index_of_max-imp_range])/(2047-index_of_max+imp_range))/hil_int-1;
    
    }
    else
    {
       
        impulsivity=((cdf[index_of_max+imp_range]-cdf[index_of_max-imp_range]));
        //printf("%f ... ",cdf[index_of_max+imp_range]-cdf[index_of_max-imp_range]);
        //printf("%f\n",hil_int);
    }
    }
    FILE * cdf_file;
    cdf_file=fopen("cdf_output.txt","w");
    for(int k = 0;k<2048;k++)
    {
        fprintf(cdf_file,"%f\n",cdf[k]);
    }
    fclose(cdf_file);
    printf("impulsivity is %f\n",impulsivity);
    clock_gettime(CLOCK_MONOTONIC,&end);
    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 
    printf("took %f per event to calculate impulsivity after getting analytic function\n",dt/do_many);


    //time to do time delay stuff
    int16_t shift=0;
    float best_corr=0;

    int16_t shifted_data[2048]={0};
    int rolled_index=0;
    int fake_shift=42;
    for(int i =0;i<2048;i++)
    {
        rolled_index=fake_shift+i;
        if(rolled_index>2048)rolled_index=rolled_index-2048;
        shifted_data[i]=real_data[rolled_index];
    }
    
    another way to speed up time delay caluclations. we can limit the range of "correct" time delays to something\
    that is physical. so instead of searching +/-1024 sample shifts, it ONLY searches t
    physical signals passing through the antennas. Max time, so one to the other
    surface antennas that is ~30 m separation or 100ns or 240sample
    helper vpols that is 1m or 5ns or 12 samples
    phased array vpols that is 3m or 15ns or 36 samples. 
    including some padding of course. it might make time-based time delays faster... tbd. for now we test the methods I think
    

    clock_gettime(CLOCK_MONOTONIC,&start);

    for(int i =0;i<30;i++)
    {
    shift=time_delay(real_data,shifted_data,&best_corr);

    }
    clock_gettime(CLOCK_MONOTONIC,&end);
    printf("shift is %i and a caluclated corr is %f\n",shift,best_corr);

    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 
    printf("took %f per event to do time based \n\n",dt/do_many);


    clock_gettime(CLOCK_MONOTONIC,&start);

    for(int i =0;i<30;i++)
    {
    shift=interp_time_delay(real_data,shifted_data,&best_corr);


    }

    clock_gettime(CLOCK_MONOTONIC,&end);
    printf("shift is %i and a caluclated corr is %f\n",shift,best_corr);

    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 
    printf("took %f per event to do interpolated time based \n\n",dt/do_many);


    clock_gettime(CLOCK_MONOTONIC,&start);

    for(int i =0;i<30;i++)
    {
        shift=fft_time_delay(real_data,shifted_data,&best_corr);


    }

    clock_gettime(CLOCK_MONOTONIC,&end);
    printf("shift is %i and a caluclated corr is %f\n",shift,best_corr);

    dt = (end.tv_sec - start.tv_sec) + 1e-9 * (end.tv_nsec - start.tv_nsec); 
    printf("took %f per event to do fft based time delay \n\n",dt/do_many);

    shift=time_delay(real_data,shifted_data,&best_corr);
    shift=interp_time_delay(real_data,shifted_data,&best_corr);
    shift=fft_time_delay(real_data,shifted_data,&best_corr);
    //to do see if time base cross correlation of fft based cross it faster
    //so can do arbitrary phased power sums (ex surface or deep w/o phased trigger)
    return 0;

*/
    return 0;
}
int test_template()
{
    int NSAMP=2048;
    fftwf_complex * in_data = fftwf_alloc_complex(NSAMP); //input data
    fftwf_complex * in_data2 = fftwf_alloc_complex(NSAMP); //input data buffer
    fftwf_complex * out_data = fftwf_alloc_complex(NSAMP); //output data

    
    fftwf_import_wisdom_from_filename("~/librno-g/helper/imp_wisdom.dat"); 
    fftwf_plan forward = fftwf_plan_dft_1d(NSAMP, in_data, out_data, FFTW_FORWARD, FFTW_MEASURE);
    fftwf_plan backward = fftwf_plan_dft_1d(NSAMP, out_data, in_data, FFTW_BACKWARD, FFTW_MEASURE);
    fftwf_export_wisdom_to_filename("~/librno-g/helper/imp_wisdom.dat");

    FILE *t_wave_file;
    int16_t template_wave[2048]={0.};

    int ret=0;
    t_wave_file=fopen("template.dat","rb");
    ret+=fread(template_wave,2,2048,t_wave_file);
    printf("%i ret\n",ret);
    fclose(t_wave_file);

    float norm1,norm2,normalization,template_corr;

    for(int sam=0;sam<2048;sam++)
    {
        in_data[sam]=template_wave[sam];//*event[0][sam];//fill array with first waveform
    }
    fftwf_execute(forward); //no normalization... gets the fft of the data and fills it to out_data bc useful
    //printf("size of comp float %i\n",sizeof(in_data[0]));
    //memcpy(in_data2,out_data,sizeof(out_data[0])*2048);//send to extra space

    for(int sam=0;sam<2048;sam++)
    {
        out_data[sam]=out_data[sam]*conj(out_data[sam]);
    }
    fftwf_execute(backward);
    norm2=cabs(in_data[0]);



    for(int sam=0;sam<2048;sam++)
    {
        in_data[sam]=template_wave[sam];//*event[0][sam];//fill array with first waveform
    }
    fftwf_execute(forward); //no normalization... gets the fft of the data and fills it to out_data bc useful
    //printf("size of comp float %i\n",sizeof(in_data[0]));
    //memcpy(in_data2,out_data,sizeof(out_data[0])*2048);//send to extra space
    norm1=0;
    
    for(int sam=0;sam<2048;sam++)
    {
        out_data[sam]=out_data[sam]*conj(out_data[sam]);
    }
    fftwf_execute(backward);
    norm1=cabs(in_data[0]);

    normalization=sqrt(norm1*norm2);

    for(int sam=0;sam<2048;sam++)
    {
        in_data[sam]=template_wave[sam];//event[j][sam];//*event[0][sam];//fill array with first waveform
    }
    fftwf_execute(forward); //no normalization... gets the fft of the data and fills it to out_data bc useful
    //printf("size of comp float %i\n",sizeof(in_data[0]));
    memcpy(in_data2,out_data,sizeof(out_data[0])*2048);//send to extra space
    for(int sam=0;sam<2048;sam++)
    {   
        in_data[sam]=template_wave[sam];//*event[1][sam]; //fill array with second waveform
    }
    fftwf_execute(forward); //no normalization... gets the fft of the data and fills it to out_data bc useful

    printf("norm 1 %f norm 2 %f\n",norm1,norm2);

    template_match(in_data, in_data2,out_data,backward,normalization,&template_corr);
    //fwrite(&template_corr,4,1,template_file);
    printf("template correlation %f\n",template_corr);

    return 0;
}
int template_match(fftwf_complex * comp1, fftwf_complex * comp2,fftwf_complex * comp3, fftwf_plan backward,float normalization, float *corr)
{
    int imax=0;
    //printf("normalization %f\n",normalization);
    float temp_corr=0;
    float abs_comp[2048]={0.};
    for(int l =0 ;l<2048;l++)
    {
        //printf("fft 1 %f ff2 %f\n",creal(comp2[l]),creal(comp3[l]));
        comp3[l]=comp3[l]*conj(comp2[l]);
    }
    fftwf_execute(backward);  //again no normalization.. so to make sense of the same waveform 1/n has to be applied to the output on x
    for(int l =0 ;l<2048;l++)
    {
            //printf("temp %f\n",abs_comp[l]);

        abs_comp[l]=cabs(comp1[l])/normalization;
        //printf("abs of iff(fft1*fft2) %f\n",cabs(comp1[l]));

        if(abs_comp[l]>temp_corr)
        {
            //printf("in here");
            imax=l;
            temp_corr=abs_comp[l]; //max correlation of the template
        }
    }

    //printf("temp %f and i %i\n\n",temp_corr,imax);
    *corr=temp_corr;


    return 0;


}

int get_time_delay_from_fft(fftwf_complex * comp1, fftwf_complex * comp2, fftwf_complex * comp3, fftwf_plan backward,int16_t *shift, float *best_corr)
{
    
    float max_cross=0;
    int i_max=0; 
    float cross[2048]={0.};
    
    for(int l =0 ;l<2048;l++)
    {
        comp3[l]=comp3[l]*conjf(comp2[l]);
    }
    //memset(a_sig+1024,0,1024);
    
    //reverse fft 
    fftwf_execute(backward);  //again no normalization.. so to make sense of the same waveform 1/n has to be applied to the output on x

    for(int i =0;i<2048;i++)
    {
        //I want to weight the shifts so no shift -> physical limit is passed and outside of that is likely ignored
        //printf("real %f, imag %f\n",creal(comp1[i]),cimag(comp1[i]));

        cross[i]=cabs(comp1[i])/(2048);
        //printf("index %i abs is %f\n",i,cross[i]);
        if(cross[i]>max_cross)

        {
            max_cross=cross[i];
            i_max=i;
        }
    }
    *best_corr=0;
    for(int i =0;i<50;i++)
    {
        //printf("%f\n",cross[i_max-(5-i)]);
        *best_corr+=cross[i_max-(25-i)]; //sum a range around the peak since power is slightly distributed
    }
    //printf("fft best shift is %i at a cross corr of %f\n",i_max,max_cross);
    *best_corr=max_cross; //pick only the peak
    //printf("sum corr = %f\n",*best_corr);
    *shift=i_max;
    //printf("i max %i and max is %f\n",i_max,max_cross);
    return 0;

}



int get_hilbert_env(fftwf_complex * comp1, fftwf_complex * comp2, fftwf_plan forward,fftwf_plan backward,float * hilbert_env,float * impulsivity)
{
    //assume data already filled into comp1 ... comp2 is for analytic signal.
    //the forward and backward plans should point between comp1 and comp2

    //fft of comp 1 will be placed in comp2
    uint16_t lc=0;
    for(lc =0 ;lc<2048;lc++)
    {
        //apply the fft->analytic signal filter on comp2
        if(lc==0) comp2[lc]=comp2[lc];
        else if(lc>0&&lc<(2048/2-1)) comp2[lc]=2*comp2[lc];
        else if(lc==2048/2-1) comp2[lc]=comp2[lc];
        else comp2[lc]=0;
    }
    
    //reverse fft so it goes from comp2 to comp1. comp1 contains the analytic signal
    fftwf_execute(backward);  //again no normalization.. so to make sense of the same waveform 1/n has to be applied to the output on x        
    
    
    //impulsivity time since we have the hilbert envelope. could also do a phase matching too probably
    //float hil_env[2048]={0.};
    float cdf[2048]={0.}; //this might need to be moved in the beginning of the scripts bc this takes up some space
    float hilbert_int=0.; //integral sum of the hilbert envelope for normalization if needed
    uint16_t imp_range=50; //the range in which the impulsiity is computed around the max of the envelope
    uint16_t index_of_max=0;
    float max=0.;
    //float impulsivity=0.;

    
    for(int lc =0;lc<2048;lc++)
    {
        //calculate the hilbert envelope from the nalytic sig in comp1
        hilbert_env[lc]=cabs(comp1[lc])/2048.;

        if(hilbert_env[lc]>max)
        {
            max=hilbert_env[lc];
            index_of_max=lc;
        }
        if(lc==0) cdf[0]=hilbert_env[0];
        else cdf[lc]=cdf[lc-1]+hilbert_env[lc];

    }

    hilbert_int=cdf[2047];

    for(int lc =0;lc<2048;lc++)
    {
        cdf[lc]=cdf[lc]/hilbert_int;
        //printf("%f\n",cdf[i]);
    }

    //printf("integral of hilbert env %f",hil_int);

    if(index_of_max<imp_range)
    {
        *impulsivity=(cdf[index_of_max+imp_range]-cdf[index_of_max]);
        //impulsivity=2*((cdf[index_of_max+imp_range]-cdf[index_of_max])/(index_of_max+imp_range))/hil_int-1;
    }
    else if(index_of_max>2047-imp_range)
    {
        *impulsivity=(cdf[2047]-cdf[index_of_max-imp_range]);
        //impulsivity=2*((cdf[2047]-cdf[index_of_max-imp_range])/(2047-index_of_max+imp_range))/hil_int-1;
    
    }
    else
    {
        *impulsivity=((cdf[index_of_max+imp_range]-cdf[index_of_max-imp_range]));
        //printf("%f ... ",cdf[index_of_max+imp_range]-cdf[index_of_max-imp_range]);
    }

    //the results of this are placed in comp2, impulsivity, hilbert envelope
    return 0;
}
int fft_time_delay(int16_t * wave1,int16_t * wave2,float * best_corr)
{
    //maybe I do squares or maybe I dont here... let me compare
    
    float wave1_sq[2048];
    float wave2_sq[2048];
    float cross[2048];

    //float * wave1_sq=malloc(4*n_sams);
    //float * wave2_sq=malloc(4*n_sams);

    for(int i =0;i<2048;i++)
    {
        // calculate as power so no <0
        wave1_sq[i]=wave1[i]*wave1[i];
        wave2_sq[i]=wave2[i]*wave2[i];
    }

    int NSAMP=2048;
    //int ntimes = 1; 


    fftwf_complex * real_data1 = fftwf_alloc_complex(NSAMP); //real time signal
    fftwf_complex * real_data2 = fftwf_alloc_complex(NSAMP); //real time signal
    
    //float * r_dats = fftwf_alloc_real(NSAMP); //real time signal
    
    fftwf_complex * fft1 = fftwf_alloc_complex(NSAMP); //complex fft from real
    fftwf_complex * fft2 = fftwf_alloc_complex(NSAMP);  //complex time for analytic signal
    
    fftwf_import_wisdom_from_filename("~/librno-g/helper/cross_wisdom.dat"); 
    
    fftwf_plan forward1 = fftwf_plan_dft_1d(NSAMP, real_data1, fft1, FFTW_FORWARD, FFTW_MEASURE);    
    fftwf_plan forward2 = fftwf_plan_dft_1d(NSAMP, real_data2, fft2, FFTW_FORWARD, FFTW_MEASURE);
    fftwf_plan backward = fftwf_plan_dft_1d(NSAMP, fft1, real_data1, FFTW_BACKWARD, FFTW_MEASURE);

    fftwf_export_wisdom_to_filename("~/librno-g/helper/cross_wisdom.dat");

    for (int j = 0; j < NSAMP; j++) 
    {
        real_data1[j] = wave1[j]; 
        real_data2[j] = wave2[j]; 

        //if(i==0)printf("%f\n",r_dats[j]);
        //r_dats[j] = real_data[j]; 

    }

    fftwf_execute(forward1); //there's no normalization applied. the first half of X is filled with the + freq comp. the second hald is empty in this case
    fftwf_execute(forward2); //there's no normalization applied. the first half of X is filled with the + freq comp. the second hald is empty in this case
    
    for(int l =0 ;l<2048;l++)
    {
        fft1[l]=fft1[l]*conjf(fft2[l]);
    }
    //memset(a_sig+1024,0,1024);
    
    //reverse fft 
    fftwf_execute(backward);  //again no normalization.. so to make sense of the same waveform 1/n has to be applied to the output on x
    float max_cross=0;
    int i_max=0;
    for(int i =0;i<2048;i++)
    {
        cross[i]=cabsf(real_data1[i]);
        if(cross[i]>max_cross)
        {
            max_cross=cross[i];
            i_max=i;
        }

    }
    //printf("fft best shift is %i at a cross corr of %f\n",i_max,max_cross);
    *best_corr=max_cross;
    return i_max;
}
int interp_time_delay(int16_t * wave1, int16_t * wave2, float * best_corr)
{
    //int o_sams=2048;
    //int new_sams=4095; //1 minus double so we don't extrapolate?

    float wave1_sq[4095]={0.};
    float wave2_sq[4095]={0.};

    
    //float * wave1_sq=malloc(4*new_sams);
    //float * wave2_sq=malloc(4*new_sams);


    for(int i =0;i<2048;i++)
    {
        int what_is=2*i;
        // calculate as power so no <0
        wave1_sq[what_is]=wave1[i]*wave1[i];
        wave2_sq[what_is]=wave2[i]*wave2[i];

        //and interpolate
        wave1_sq[what_is-1]= wave1_sq[what_is-2]+ (wave1_sq[what_is]-wave1_sq[what_is-2])/2;
        wave2_sq[what_is-1]= wave2_sq[what_is-2]+ (wave2_sq[what_is]-wave2_sq[what_is-2])/2;
        
    }

    float cross_corr=0;
    //what about shifting it by one
    int guess_shift=0;
    //int rol;
    *best_corr=0;
    int best_shift=0;
    int isam=0;
    int ac1=0;
    int ac2=0;
    for(guess_shift=0;guess_shift<4095-1;guess_shift++)
    {
        cross_corr=0;
        for(isam=0;isam<4095;isam++)
        {

            ac1=isam;
            ac2=isam+guess_shift;
            if(ac2>4095-1)ac2=ac2-4095; //if outside range roll it back into
            cross_corr+=wave1_sq[ac1]*wave2_sq[ac2];
            

            //cross_corr+=wave1[i]*wave2[2047-i-shift];//only shift the second. and let the rol find the right index

        }
        if(cross_corr>*best_corr)
        {
            *best_corr=cross_corr;
            best_shift=guess_shift;
        }
    }

    //free(wave1_sq);
    //free(wave2_sq);

    //printf("interpolated best shift is %i at a cross corr of %f\n",best_shift,*best_corr);
    return (4095-best_shift)/2;



}

int time_delay(int16_t * wave1,int16_t * wave2, float * best_corr)
{

    //check to make sure non nulls?
    
    //int n_sams=2048;

    float wave1_sq[2048];
    float wave2_sq[2048];

    //float * wave1_sq=malloc(4*n_sams);
    //float * wave2_sq=malloc(4*n_sams);



    for(int i =0;i<2048;i++)
    {
        // calculate as power so no <0
        wave1_sq[i]=wave1[i]*wave1[i];
        wave2_sq[i]=wave2[i]*wave2[i];
    }
    //need to abs or ^2 or use hilbert envelope. <zero cross corrs could cause issues?
    float cross_corr=0;
    //start off by calculating the zero shift inner product using time based calc
    int isam=0;
    int ac1=0;
    int ac2=0;
    //for(isam=0;isam<2048;isam++)
    //{
    //    ac1=isam;
    //    ac2=isam;
    //    if(ac2>2048-1)ac2=ac2-2048; //IM NOT DOING CONVOLUTION!!! ORDER MATTERS SO DONT REVERSE
    //    cross_corr+=wave1_sq[ac1]*wave2_sq[ac2];
    //}
    
    //printf("auto correlation (zero shift) value=%f\n",cross_corr);

    //what about shifting it by one
    int guess_shift=0;
    //int rol;
    cross_corr=0.;
    *best_corr=0;
    int best_shift=0;
    for(guess_shift=0;guess_shift<2048-1;guess_shift++)
    {
        cross_corr=0;
        for(isam=0;isam<2048;isam++)
        {

            ac1=isam;
            ac2=isam+guess_shift;
            if(ac2>2048-1)ac2=ac2-2048; //if outside range roll it back into
            cross_corr+=wave1_sq[ac1]*wave2_sq[ac2];
            

            //cross_corr+=wave1[i]*wave2[2047-i-shift];//only shift the second. and let the rol find the right index

        }
        if(cross_corr>*best_corr)
        {
            
            *best_corr=cross_corr;
            best_shift=guess_shift;
        }
    }

    //free(wave1_sq);
    //free(wave2_sq);

    //printf("time based best shift is %i at a cross corr of %f\n",best_shift,*best_corr);
    return 2048-best_shift;
    //return 0;
}


int vector_rms(int16_t * waveform, float * rms)
{
    //some notes:
    //I did one uint16x4_t, and the time didn't really change to using 4...
    //this method is slower than just using the normal types. don't know why...
    //maybe intrinsics for this purpose due to moving data around and then converting.
    #if defined(__arm__)


    static int32_t temp_rms[2048]={};
    static int16x4_t a;


    *rms=0.;
    //bfloat16x8_t converted;
    static int32x4_t vect_rms0;

    static int32x4_t sum1;



    //the lab4d 12bit digitizier->0xfff*0xfff <28bits . these uint16_t should be smaller than the max of uint32_t
    uint16_t count;
    uint16_t small_count;
    for(count=0;count<512;count++)
    {
        //load in 4 samples at a time... 2048/4samps=512
        a=vld1_s16(waveform+count*4);

        
        //converted=vcvtq_b16_u16(a);

        //multiply arrays
        vect_rms0=vmull_s16(a,a);


        //sum1=vadd

        
        //sum total
        //*rms+=vsum_u16(vect_rms);
        //temp_rms=vpadd_u16(vect_rms);
        //vst1q_u32(temp_rms+count*4,vect_rms);
        //sum the registers to the float rms
        for(small_count=0;small_count<4;small_count++)
        {
            *rms+=vect_rms0[small_count];
        }

        //multiply vector with scalar vmulq_n_u16(a,#)
    }

    
    //*rms+=temp_rms;
    //divide by samples 
    //*rms=*rms/(2048.*2048.);

    *rms=sqrt(*rms/2048.);
    return 0;
    #endif
    float_rms(waveform,rms);
    //printf("arm not used, use old\n");
    return 1;
}

int calc_snr(int16_t * waveform,float * rms,float * snr)
{
    uint16_t i;
    int16_t max_adc=0;
    int16_t min_adc=0;

    for(i=0;i<2048;i++)
    {
        if(waveform[i]>max_adc) max_adc=waveform[i];
        if(waveform[i]<min_adc) min_adc=waveform[i];
    }

    *snr = (float)(max_adc-min_adc)/(2*(*rms));
    return 0;
}

int float_rms(int16_t * waveform, float * rms)
{

    float temp_rms=0;
    *rms=0;
    //uint8_t ichan;
    uint16_t isam;
    //printf("calculating rms\n");
    //sum sqaures
    float temp_val=0;
    for(isam=0;isam<2048;isam++)
    {
        //printf("rms %f\n",temp_rms);
        //printf("wave %i\n",waveform[isam]);
        //printf("val %i\n",waveform[isam]);
        temp_val=waveform[isam];
        temp_rms+=temp_val*temp_val;
        //fast and dirty power rms[ichan]+=waveforms[ichan][isam]<<2;
    }
    //printf("went through waveform\n");
    //mean square
    *rms=temp_rms/2048.;//integer division ok
    //root
    *rms=sqrt(*rms);
    
    //print result
    //printf("calculated rms as %f\n",*rms);
    return 0;
}

int old_rms(int16_t * waveform, float * rms)
{   
    *rms=0;

    uint16_t isam;
    uint32_t old_rms=0;
    //sum sqaures
    for(isam=0;isam<2048;isam++)
    {
        old_rms+=waveform[isam]*waveform[isam];
    }
    //mean square
    old_rms=old_rms/2048;//integer division ok
    //root
    *rms=sqrt(old_rms);//now converts to float I think

    
    return 0;
    //print result
}



