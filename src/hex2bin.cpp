// 2018-08-19 LLW and AS program to parse dsl format hex strings on
// STDINand output binary on STDOUT
#include <stdio.h>
#include <string.h>

int main( int argc, const char* argv[] )
{
  char instr[1024];
  char * the_good_stuff;
  char * gets_status;
  int nlines = 0;
  int  sscanf_status;
  unsigned char data_byte;
  int lines = 0;
  long int out_bytes;

  // hello
  fprintf(stderr, "Usage: cat INFILE_DSLHEX.HEX  | hex2bin FE81 > outfile.bin\n");
  fprintf(stderr, "           where FE81 is the hext start sequence of the hex string\n");
  fprintf(stderr, "File %s compiled on %s %s\n",__FILE__,__TIME__,__DATE__ );
  
  if (argc < 2)
    return(1);

  //  fprintf(stderr,"Scanning for hex start sequence \"%s\"\n",argv[1]); 

  do
    {
      // get the next string, returns arg if success, NULL if fail
      gets_status = gets(instr);

      // if we got a string, parse it
      if(gets_status != NULL)        
	{

	  // count lines
	  lines++;
	  //	  fprintf(stderr,"Got line %d: %s\r", lines, instr);
	  
	  // scan for hex start sequence
	  the_good_stuff = strstr(instr, argv[1]);

	  // check to see if we got the desired sequence
	  if (the_good_stuff != NULL)
	    {
	    // decode the hex, two hex chars (one  binary byte) at a time
	    do
	      {
		sscanf_status = sscanf(the_good_stuff, "%02hhX",&data_byte);

		//		fprintf(stderr,"Got %02X\n",data_byte);

		// status gives the number of args actuall assigned
		if(sscanf_status == 1)
		  {
		    // write byte to stdout
		    putchar(data_byte);

		    // count outpout chars
		    out_bytes++;

		    // increment to point to next char of good stuff
		    the_good_stuff = the_good_stuff+2;
		  }
	      }
	    while(sscanf_status>0);
	    } 
	}
    }
  while (gets_status != NULL);
  //  while ((gets_status != NULL) && (lines < 1));

  fprintf(stderr,"Read %d lines, wrote %ld bytes\n", lines, out_bytes); 

}  
