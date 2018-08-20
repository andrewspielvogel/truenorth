// 2018-08-19 LLW throttle stdin to stdout
#include <stdio.h>
#include <string.h>
#include <unistd.h>


int main( int argc, const char* argv[] )
{
  int bytes_per_sec;
  int status;
  unsigned char inchar;
  int i;

  // hello
  fprintf(stderr, "Usage: cat INFILE | throttle 1024  | socat....\n" );
  fprintf(stderr, "           to throttle at 1024 bytes/sec\n");
  fprintf(stderr, "File %s compiled on %s %s\n",__FILE__,__TIME__,__DATE__ );

	  
  if (argc < 2)
    return(1);

  status = sscanf(argv[1],"%d", &bytes_per_sec);


  if((status != 1) || (bytes_per_sec <= 0))
    return(1);

  fprintf(stderr,"Throttling at %d bytes/sec.\n", bytes_per_sec);
  
  do
    {

      for(i=0; i< bytes_per_sec; i++)
	{
	  inchar = getchar();
	  if (inchar != EOF)
	    putchar(inchar);
	}

      fflush(stdout);
      
      sleep(1);
    }
  while (inchar != EOF);


}  
