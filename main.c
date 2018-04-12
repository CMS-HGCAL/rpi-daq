#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <time.h>


#define MODE_READ 0
#define MODE_WRITE 1
#define MODE_SET   1      // redundant
#define MODE_CLR 2
#define MODE_INPUT_READ 3

#define PULL_UP 0
#define PULL_DOWN 1
#define NO_PULL 2

#define GPIO_BEGIN 0
#define GPIO_END 1
#define NO_ACTION 2

static unsigned char BusMode;         // global to remember status of gpio lines (IN or OUT)

#define STpin RPI_V2_GPIO_P1_12
#define RWpin RPI_V2_GPIO_P1_11
#define AD0pin RPI_V2_GPIO_P1_07
#define AD1pin RPI_V2_GPIO_P1_13
#define AD2pin RPI_V2_GPIO_P1_15
#define AD3pin RPI_V2_GPIO_P1_29
#define D0pin RPI_V2_GPIO_P1_37
#define D1pin RPI_V2_GPIO_P1_36
#define D2pin RPI_V2_GPIO_P1_22
#define D3pin RPI_V2_GPIO_P1_18
#define D4pin RPI_V2_GPIO_P1_38
#define D5pin RPI_V2_GPIO_P1_40
#define D6pin RPI_V2_GPIO_P1_33
#define D7pin RPI_V2_GPIO_P1_35
#define ACKpin RPI_V2_GPIO_P1_16


/*****************************************************/
/* define commands for Master FPGA */
/*****************************************************/
#define CMD_IDLE         0x80
#define CMD_RESETPULSE   0x88
#define CMD_WRPRBITS     0x90
#define CMDH_WRPRBITS    0x12 
#define CMD_SETSTARTACQ  0x98
#define CMD_STARTCONPUL  0xA0
#define CMD_STARTROPUL   0xA8
#define CMD_SETSELECT    0xB0
#define CMD_RSTBPULSE    0xB8
#define CMD_READSTATUS   0xC0
#define CMDH_READSTATUS  0x18
#define CMD_LOOPBRFIFO   0xF0
#define CMDH_LOOPBRFIFO  0x1E
#define CMD_LOOPBACK     0xF8
#define CMDH_LOOPBACK    0x1F
#define DAC_HIGH_WORD    0xFF
#define DAC_LOW_WORD     0x0F
#define TRIGGER_DELAY    0x1F// 0x00 to 0x07
#define TRIGGER_DELAY    0x1F

// Calibration String enabling Test Pulse on Ch. 30
unsigned char prog_string_sign_inj[48] = {0xDA,0xA0,0xF9,0x32,0xE0,0xC1,0x2E,0x10,0x98,0xB0,
0x40,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x1F,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xE9,0xD7,0xAE,0xBA,0x80,0x25};
				   				
// No signal injection TOA 270			 
unsigned char prog_string_no_sign[48] = {0xDA,0xA0,0xF9,0x32,0xE0,0xC1,0x2E,0x10,0x98,0xB0,
0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xE9,0xD7,0xAE,0xBA,0x80,0x25};

// Long Calibration String
unsigned char long_prog_string[192] = {
					0xDA, 0xA0, 0xF9, 0x32, 0xE0, 0xC1, 0x2E, 0x10, 0x98, 0xB0, 
 					0x40, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xE9, 0xD7, 0xAE, 0xBA, 0x80, 0x25,
 					0xDA, 0xA0, 0xF9, 0x34, 0xE0, 0xC1, 0x2E, 0x10, 0x98, 0xB0, 
 					0x40, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xE9, 0xD7, 0xAE, 0xBA, 0x80, 0x25,
 					0xDA, 0xA0, 0xF9, 0x36, 0xE0, 0xC1, 0x2E, 0x10, 0x98, 0xB0, 
 					0x40, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xE9, 0xD7, 0xAE, 0xBA, 0x80, 0x25,
 					0xDA, 0xA0, 0xF9, 0x38, 0xE0, 0xC1, 0x2E, 0x10, 0x98, 0xB0, 
 					0x40, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
 					0xFF, 0xFF, 0xE9, 0xD7, 0xAE, 0xBA, 0x80, 0x25};
			
// Long Calibration String
unsigned char zero_padding_string[192] = {	0x00};
			
			
#define RAWSIZE 30787
#define nSCA 15

int set_bus_init();
int set_bus_read_mode();
int set_bus_write_mode();
int send_command(unsigned char);
int read_command(void);
int read_usedwl();
int read_usedwh();
int write_local_fifo(unsigned char );
int read_local_fifo();
unsigned char return_string[48], long_return_string[192];
unsigned char raw[RAWSIZE];
unsigned int ev[4][1924];
unsigned int dati[4][128][nSCA];

int read_raw(){
	int i, t;
	for (i = 0; i < 30785; i = i+1){
		t =read_local_fifo();
		raw[i] = (unsigned char) (t & 255);
	}
}

int decode_raw(){
    int i, j, k, m;
    unsigned char x;
	unsigned int t;
	unsigned int bith, bit11, bit10, bit9, bit8, bit7, bit6, bit5, bit4, bit3, bit2, bit1, bit0;
	for(i = 0; i < 1924; i = i+1){
		for(k = 0; k < 4; k = k + 1){
			ev[k][i] = 0;
		}
   	}
    for(i = 0; i < 1924; i = i+1){
        for (j=0; j < 16; j = j+1){
            x = raw[1 + i*16 + j];
            x = x & 15;
            for (k = 0; k < 4; k = k + 1){
            	ev[k][i] = ev[k][i] | (unsigned int) (((x >> (3 - k) ) & 1) << (15 - j));
            }
        }
   	}
  
/*****************************************************/
/*    Gray to binary conversion                      */
/*****************************************************/
   	for(k = 0; k < 4 ; k = k +1 ){
   		for(i = 0; i < 128*nSCA; i = i + 1){
   			bith = ev[k][i] & 0x8000;
   			t = ev[k][i] & 0x7fff;
   			bit11 = (t >> 11) & 1;
        	bit10 = bit11 ^ ((t >>10) &1);
        	bit9 = bit10 ^ ((t >>9) &1);
        	bit8 = bit9 ^ ((t >>8) &1);
        	bit7 = bit8 ^ ((t >>7) &1);
        	bit6 = bit7 ^ ((t >>6) &1);
        	bit5 = bit6 ^ ((t >>5) &1);
        	bit4 = bit5 ^ ((t >>4) &1);
        	bit3 = bit4 ^ ((t >>3) &1);
        	bit2 = bit3 ^ ((t >>2) &1);
        	bit1 = bit2 ^ ((t >>1) &1);
        	bit0 = bit1 ^ ((t >>0) &1);
        	ev[k][i] =  bith | ((bit11 << 11) + (bit10 << 10) + (bit9 << 9) + (bit8 << 8) + (bit7 << 7) + (bit6 << 6) + (bit5 << 5) + (bit4 << 4) + (bit3  << 3) + (bit2 << 2) + (bit1  << 1) + bit0);
        }
    }
}

int format_channels(){
    int chip, hit, ch;
    for(chip =0; chip < 4; chip = chip +1 ){
        for(ch = 0; ch < 128; ch = ch +1 ){
            for(hit = 0 ; hit <nSCA ; hit = hit +1){
                dati[chip][ch][hit] = ev[chip][hit*128+ch] & 0x0FFF;
            }
        }
    }
    return(0);
}
    
int main()
{
int 	res, temp, i, j, n, k, count, correct, error;
int 	ch, sample, chip;
int 	maxevents = 10;
int 	dac_ctrl = 0;
int 	dac_fs = 1920;
struct tm *info;
char 	buffer[80];
char 	fname [160];
char 	dirname[] = "./Data/";	
char 	pulse_inj[128], pulse_sweep[128], acquisition_type[128], long_conf_switch[128], default_configuration_switch[128], default_configuration_acqt_switch[128];
char	program_switch[128], loopback_switch[128], data_acquisition_switch[128], full_checks_switch[128], triggered_acqt[128];
char 	instr [1024];
long 	delay1, delay2, delay3, delay4, delay5, delay6; 
FILE *fraw;
FILE *fout;
time_t rawtime;
bool saveraw;

delay1 = 100;
delay2 = 0;
delay3 = 3000;
delay4 = 100;
delay5 = 0;
srand(time(NULL));
count = 0;
correct = 0;
error = 0;

/***********************************************************************/
/*  Initialize RPI */
/***********************************************************************/
if (!bcm2835_init())
	return 1;
set_bus_init();
bcm2835_gpio_fsel(RWpin, BCM2835_GPIO_FSEL_OUTP);

/***********************************************************************/
/***** Input Parameters ***/
/***********************************************************************/

printf("Full Checks ? [Y/N] ");
scanf ("%s",full_checks_switch);

if(full_checks_switch[0] == 'N' | full_checks_switch[0] == 'n'){
	printf("\nFPGA Loopback check ? [Y/N] ");
	scanf ("%s",loopback_switch);

	printf("\nLong Configuration String ? [Y/N] ");
	scanf ("%s",long_conf_switch);
	
	printf("\nProgramming Loop? [Y/N] ");
	scanf("%s", program_switch);
	
	printf("\nRead Defaulf Configuration String? [Y/N] ");
	scanf("%s", default_configuration_switch);
}
else {
	loopback_switch[0] = 'y';
	long_conf_switch[0] = 'y';
	program_switch[0] = 'n';
	default_configuration_switch[0] = 'y';
}
if(program_switch[0] != 'Y' & program_switch[0] != 'y' & program_switch[0] != 'N' & program_switch[0] != 'n'){
	printf("\nInvalid Selection.");	
	exit(0);
}
printf("\nData Acquisition? [Y/N] ");
scanf("%s", data_acquisition_switch);
if(data_acquisition_switch[0] != 'Y' & data_acquisition_switch[0] != 'y' & data_acquisition_switch[0] != 'N' & data_acquisition_switch[0] != 'n'){
	printf("\nInvalid Selection.");	
	exit(0);
}
if(program_switch[0] == 'N' | program_switch[0] == 'n'){
	if(data_acquisition_switch[0] == 'Y' | data_acquisition_switch[0] == 'y'){
		printf("\nHow many events ? ");
		scanf ("%d",&maxevents);
		printf("\nSave raw data ? [Y/N] ");
		scanf ("%s",instr);
		saveraw = false;
		if(instr[0] == 'y' | instr[0] == 'Y')
			saveraw = true;
		printf("\nExternal Trigger for Acquisitions? [Y/N] ");
		scanf("%s", triggered_acqt);	
		printf("\nUse default configuration for acquisitions? [Y/N] ");
		scanf ("%s",default_configuration_acqt_switch);
		if((default_configuration_acqt_switch[0] == 'N' | default_configuration_acqt_switch[0] == 'n') & (triggered_acqt[0] == 'N' | triggered_acqt[0] == 'n' )){	
			printf("\nExternal Pulse Injection? [Y/N] ");
			scanf("%s", pulse_inj);
			if(pulse_inj[0] != 'Y' & pulse_inj[0] != 'y' & pulse_inj[0] != 'N' & pulse_inj[0] != 'n'){
				printf("\nInvalid Selection.");	
				exit(0);
			}
			if(pulse_inj[0] == 'N' | pulse_inj[0] == 'n'){
				printf("\nUse fixed Acquisition Window? [Y/N] ");
				scanf("%s", acquisition_type);
				if(acquisition_type[0] != 'Y' & acquisition_type[0] != 'N' & acquisition_type[0] != 'y' & acquisition_type[0] != 'n'){
					printf("\nInvalid Selection.");	
					exit(0);
				}
			}
			if(pulse_inj[0] == 'Y' | pulse_inj[0] == 'y'){
				acquisition_type[0] = 'n';
				printf("\nSweep Pulse Amplitude? [Y/N] ");
				scanf("%s", pulse_sweep);
				if(pulse_sweep[0] != 'Y' & pulse_sweep[0] != 'N' & pulse_sweep[0] != 'y' & pulse_sweep[0] != 'n'){
					printf("\nInvalid Selection.");	
					exit(0);
				}
			}	
			else{
					pulse_sweep[0] = 'n';
			}
		}
	}
}
else{
printf("\nHow many programming cycles? ");
scanf ("%d",&maxevents);
}
	


/***********************************************************************/
/* empty local fifo by forcing extra reads, ignore results */
/***********************************************************************/
printf("\n####################################################\n");
printf("Empting local FIFO");
printf("\n####################################################\n");
i = 0;
j = 0;
for(i=0; i < 34000; i = i+1){
	res = read_local_fifo();	
}
res = set_trigger_delay(TRIGGER_DELAY);

/***********************************************************************/
/* Default Configuration String */
/***********************************************************************/
if(default_configuration_switch[0] == 'Y' | default_configuration_switch[0] == 'y'){
	printf("\n####################################################\n");
	printf("Default configuration check");
	printf("\n####################################################\n");
	res = send_command(CMD_RSTBPULSE);
	res = send_command(CMD_SETSELECT | 1);
	res = send_command(CMD_RSTBPULSE);
	read_configuration_string(zero_padding_string, long_return_string); 
	printf("\nDefault configuration read from SK2CMS chips\n");
		for(i = 0; i < 48*4; i = i + 1){
			printf("%x ",long_return_string[i]);
		}	
	read_configuration_string(zero_padding_string, long_return_string); 
	printf("\nSlow control register after zero padding\n");
		for(i = 0; i < 48*4; i = i + 1){
			printf("%x ",long_return_string[i]);
		}
	printf("\n");
	res = send_command(CMD_RSTBPULSE);	
	res = send_command(CMD_SETSELECT);	
}

/***********************************************************************/
/* FPGA Checks */
/***********************************************************************/
if(loopback_switch[0] == 'Y' | loopback_switch[0] == 'y'){
	printf("\n####################################################\n");
	printf("FPGA LoopBack Check");
	printf("\n####################################################\n");
	for(k = 0; k < 1000000; k = k + 1){
		n = rand() & 0x7;
		res = send_command(CMD_LOOPBRFIFO | n);
		temp = read_command();
		if(temp  == CMD_LOOPBRFIFO | n){
			count = count + 1;
			correct = correct + 1;}
		else{
			error = error + 1;
			printf("\n %x %x", temp,  CMD_LOOPBRFIFO | n);
		}
		if(count%10000==0){
			printf("%s", "*");
			fflush(stdout);
		}
	}

printf("\n%s", "Results:");
printf("\n%s", "Correct:");
printf("\n\t%i", correct);
printf("\n%s", "Errors:");
printf("\n\t%i\n", error);
}

/*****************************************************/
/*Long configuration                     */
/*****************************************************/
if(long_conf_switch[0] == 'Y' | long_conf_switch[0] == 'y' ){
	printf("\n####################################################\n");
	printf("Slow control configuration with long configuration string");
	printf("\n####################################################\n");
	time(&rawtime);
	info = localtime(&rawtime);
	strftime(buffer,80,"RUN_%d%m%y_%H%M", info);
	strcpy(fname, dirname);
	strcat(fname, buffer);
	strcat(fname,"_configuration");
	strcat(fname,".txt");
	printf("\nFilename will be %s\n",fname);
	fout = fopen(fname, "w");
    res = send_command(CMD_RSTBPULSE);
    res = send_command(CMD_SETSELECT | 1);
    fprintf(fout,"Configuration used for SK2\n");
    printf("\nConfiguration used for SK2\n");
	for(i = 0; i < 192; i = i + 1){
		fprintf(fout,"%x ",long_prog_string[i]);
		printf("%x ",long_prog_string[i]);
	}
    fprintf(fout,"\nConfiguration read from SK2\n");	
   	printf("\nConfiguration read from SK2\n");	
    progandverify48_4chips(long_prog_string, long_return_string); 
	for(i = 0; i < 192; i = i + 1){
		fprintf(fout,"%x ",long_return_string[i]);
		printf("%x ",long_return_string[i]);
	}
    
	if(strcmp(long_return_string, long_prog_string) == 0){			
		printf("\nConfiguration strings match!!\n");
	}
    res = send_command(CMD_SETSELECT);	
    usleep(10000);
    printf("\nLong Configuration finished!\n");
    fclose(fout);
}
    
/***********************************************************************/
/* Normal Acquisition */
/***********************************************************************/
if(program_switch[0] == 'N' | program_switch[0] == 'n' ){
	
	if(data_acquisition_switch[0] == 'Y' | data_acquisition_switch[0] == 'y'){
		printf("\n####################################################\n");
		printf("Data Acquisition");
		printf("\n####################################################\n");
		/***********************************************************************/
		/* Make up a file name for data */
		/***********************************************************************/
		time(&rawtime);
		info = localtime(&rawtime);
		strftime(buffer,80,"RUN_%d%m%y_%H%M", info);
		strcpy(fname, dirname);
		strcat(fname, buffer);
		strcat(fname,".txt");
		printf("\nFilename will be %s\n",fname);
		fout = fopen(fname, "w");
		fprintf(fout,"Total number of events: %d, External Pulse: %s, Fixed Length Acquisition: %s, delays: %li %li %li %li %li ",maxevents, pulse_inj, acquisition_type, delay1, delay2, delay3, delay4, delay5);
		fprintf(fout,"\n%s\n####################################################\n",buffer);
		strcpy(fname, dirname);
		strcat(fname, buffer);
		strcat(fname,".raw.txt");
		if(saveraw){
			printf("Raw filename will be %s\n",fname);
			fraw = fopen(fname, "w");
		}
		/*****************************************************/
		/*             set configuration                     */
		/*****************************************************/
		res = send_command(CMD_RSTBPULSE);
		res = send_command(CMD_SETSELECT | 1);
		res = send_command(CMD_RSTBPULSE);
		/*printf("\n\n################ Setup #############################\n");
		printf("Save Raw Data: %s\nExternal Pulse Injection: %s\nFixed Acquisition Window: %s\nAmplitude Sweep of Injected Signal: %s\n", instr, pulse_inj, acquisition_type,pulse_sweep);
		printf("####################################################\n");*/
		if(default_configuration_acqt_switch[0] == 'N' | default_configuration_acqt_switch[0] == 'n'){
			if(pulse_inj[0] == 'N' | pulse_inj[0] == 'n'){
				fprintf(fout,"Configuration used for SK2\n");
					for(i = 0; i < 48; i = i + 1){
						fprintf(fout,"%x ",prog_string_no_sign[i]);
					}
				fprintf(fout,"\nConfiguration read from SK2\n");	
				progandverify48(prog_string_no_sign, return_string); 
					for(i = 0; i < 48; i = i + 1){
						fprintf(fout,"%x ",return_string[i]);
					}
				fprintf(fout,"\n");	
			}
			else{
				fprintf(fout,"Configuration used for SK2\n");
					for(i = 0; i < 48; i = i + 1){
						fprintf(fout,"%x ",prog_string_sign_inj[i]);
					}
				fprintf(fout,"\nConfiguration read from SK2\n");	
				progandverify48(prog_string_sign_inj, return_string); 
					for(i = 0; i < 48; i = i + 1){
						fprintf(fout,"%x ",return_string[i]);
					}
				fprintf(fout,"\n");
			}
		}
		res = send_command(CMD_SETSELECT);	
		usleep(10000);
		printf("\nFinished Configuration\n");
			
		/*****************************************************/
		/*                  do the work                     */
		/*****************************************************/
		printf("\nStart events acquisition\n");
		n = 0;
		for(i = 0; i < maxevents; i = i +1){
			if(pulse_sweep[0] == 'Y' | pulse_sweep[0] == 'y'){ 
				dac_ctrl = dac_ctrl + dac_fs/maxevents;
				res = set_dac_high_word((dac_ctrl & 0xFF0)>>4);
				res = set_dac_low_word(dac_ctrl & 0x00F);
			}
			else{
				res = set_dac_high_word(DAC_HIGH_WORD);
				res = set_dac_low_word(DAC_LOW_WORD);	
			}
			res = send_command(CMD_RESETPULSE);
			usleep(delay1);
			if(acquisition_type[0] == 'Y' | acquisition_type[0] == 'y')
				res = fixed_acquisition();
			else {	
				res = send_command(CMD_SETSTARTACQ | 1);
				usleep(delay2);
				if(triggered_acqt[0] == 'Y' | triggered_acqt[0] == 'y'){
					res = instrumental_trigger();
					//res = send_command(CMD_STARTCONPUL);

					int word = 0;
					while(true){
					  if (word > 3770) {
					    word = (read_usedwh() << 8) | read_usedwl();
					    //printf("Done %i \n",word);
					    break;
					  }
					  
					  word = (read_usedwh() << 8) | read_usedwl();
					  //printf(" %i ",word);

					  usleep(10);
					}
					//usleep(delay3);
					//res = send_command(CMD_STARTROPUL);
					//usleep(delay4);
					/*
					usleep(delay3);
					usleep(delay4);
					*/
				}
				else{
					if(pulse_inj[0] == 'N' | pulse_inj[0] == 'n') 
						res = send_command(CMD_SETSTARTACQ);  /* <<<+++   THIS IS THE TRIGGER */
					else	
						calib_gen();
						
					res = send_command(CMD_STARTCONPUL);
					usleep(delay3);
					res = send_command(CMD_STARTROPUL);
					usleep(delay4);
					
				}
			}
			res = read_raw();	
			if(saveraw)
				fwrite(raw, 1, sizeof(raw), fraw);

		/*****************************************************/
		/*         convert raw to readable data             */
		/*****************************************************/
			res = decode_raw();
		/*****************************************************/
		/* do some verification that data look OK on one chip*/
		/*****************************************************/
			chip= 1;
			for(k = 0; k < 1664; k = k + 1){
				if((ev[chip][k] & 0x8000 ) == 0){
					printf("Wrong MSB at %d %x \n",k,ev[chip][k]);
				}
			}
		/*****************************************************/
		/*           final convert to readable stuff         */
		/*****************************************************/
			res = format_channels();
			if(i % 10 == 0){
				printf("%d ", i);
				fflush(stdout);
			}
		/*****************************************************/
		/*             write event to data file              */
		/*****************************************************/
			for(chip = 0; chip < 4; chip = chip + 1){
				fprintf(fout, "Event %d Chip %d RollMask %x \n",i, chip, ev[chip][1920]);
				for(ch =0; ch < 128; ch = ch +1){
					for (sample=0; sample < nSCA; sample = sample +1){
						fprintf(fout, "%d  ", dati[chip][ch][sample]);
					}
					fprintf(fout, "\n");
				}	
			}
			usleep(delay5);
			}
			printf("\nChip 0 - Roll Position: %x \n",ev[0][1920]);
			printf("Chip 0 - Global Time Stamp 14 MSB: %x \n",ev[0][1921]);
			printf("Chip 0 - Global Time Stamp 12 LSB + 1 extra bit: %x \n",ev[0][1922]);
			printf("Chip 0 - ChipId: %x \n",ev[0][1923]);
			printf("\nChip 1 - Roll Position: %x \n",ev[1][1920]);
			printf("Chip 1 - Global Time Stamp 14 MSB: %x \n",ev[1][1921]);
			printf("Chip 1 - Global Time Stamp 12 LSB + 1 extra bit: %x \n",ev[1][1922]);
			printf("Chip 1 - ChipId: %x \n",ev[1][1923]);
			printf("\nChip 2 - Roll Position: %x \n",ev[2][1920]);
			printf("Chip 2 - Global Time Stamp 14 MSB: %x \n",ev[2][1921]);
			printf("Chip 2 - Global Time Stamp 12 LSB + 1 extra bit: %x \n",ev[2][1922]);
			printf("Chip 2 - ChipId: %x \n",ev[2][1923]);
			printf("\nChip 3 - Roll Position: %x \n",ev[3][1920]);
			printf("Chip 3 - Global Time Stamp 14 MSB: %x \n",ev[3][1921]);
			printf("Chip 3 - Global Time Stamp 12 LSB + 1 extra bit: %x \n",ev[3][1922]);
			printf("Chip 3 - ChipId %x \n",ev[3][1923]);
			if(saveraw)
				fclose(fraw);
			return(0);     
}
}
/***********************************************************************/
/* Programming Loop */
/***********************************************************************/	
else{
	printf("\n####################################################\n");
	printf("Programming Loop");
	printf("\n####################################################\n");
	/*****************************************************/
	/*             set configuration                     */
	/*****************************************************/
	for(i = 0; i < maxevents; i = i +1){
		res = send_command(CMD_RSTBPULSE);
		res = send_command(CMD_SETSELECT | 1);
		progandverify48(prog_string_no_sign, return_string); 
		if(i % 20 == 0){
			printf("%d ", i);
			fflush(stdout);
		}
	}  
	res = send_command(CMD_SETSELECT);	
	usleep(10000);
	printf("\nConfiguration Loop terminated\n");
}
}
