function err = tn_opt_fmincon_llw(p)
% 
% 2018-08-29 LLW For numerical optmization of KVH gyro bias with fminsearch
% this file is based upon att_process_script.sh by Andrew Spielvogel
%
% p is a Nx1 set of algorithm parameters, initially just angular rate bia
%
% expects a global phins data structure named 'phins' is loaded in workspace
%
% returns nonnegatve scalar err 
%  

  % if global data structure phins has been loaded, then we will load it
  global phins;

  if(1==exist('phins'))
    fprintf(1,'tn_opt: Phins data structure exists in Workspace\n');
  else 
    fprintf(1,'tn_opt: Phins data structure does not exist in Workspace\n');
  end 
  

  % construct command line for gen_config_file.py
  TrueNorth_Package_dir = '/home/spiels/catkin_ws/src/truenorth'

  % exp dir with subdirectories /kvh, /phine, and /proc
  EXP_dir = '/home/spiels/log/JHUROV/dives_10hz/JHUROV/'
  %EXP_dir = '/home/spiels/log/sim/'
  %EXP_dir = '/home/spiels/2018-Sentry/cruise_data/dives_10hz/sentry494/'
  %EXP_dir = '/home/spiels/exp/dive2/'

  % name of expt file with no suffix such as 2018_08_21_12_45'
  LOG_in_fn  = 'dive10-11'
  LOG_in_fn  = 'dive10-08'
  %LOG_in_fn  = '2018_12_05_18_17'
  %LOG_in_fn  = '2018_08_07'
  %LOG_in_fn  = 'exp6'
  %LOG_in_fn  = 'dive_10hz_bottom'



  LOG_out_fn = ['dive_10hz_expt06']
  %LOG_out_fn = ['exp4_exp1']
  %LOG_out_fn = ['sentry_exp1']

  HZ = '10'
  
  % algorithm parameters NO SPACES BETWEEN NUMBERS OR COMMAS
  rpy_align=  '[-1.5798,0.0573,1.5712]';
  rpy_align=  '[-1.5708,0,1.5708]';
  %rpy_align= '[0,0,0]';

  % rpy_ro=     '[0.0,0.0,4.01]';
  %  2018-09-06 LLW 0.5 radian initial heading error
  rpy_ro=     '[0.0,0.0,-0.15]'
  rpy_ro=     '[0.0,0.0,0.6]'
  rpy_ro=     '[0.0,0.0,0.2]'

  % gains appearing in the attitude observer
  k_g=        '[1,1,1]';
  %k_g=           sprintf('[%.12f,%.12f,%.12f]',p(5),p(5),p(5));  
  k_north=    '[1,1,1]';
  %k_north=       sprintf('[%.12f,%.12f,%.12f]',p(6),p(6),p(6));  
  

  % gains appearing in the bias estimator
  %k_acc=      '[10.0,10.0,10.0]';
  k_acc=         sprintf('[%.12f,%.12f,%.12f]',p(1),p(1),p(1));  
  % k_E_n=      '[0.0001,0.0001,0.0001]';
  k_E_n=         sprintf('[%.12f,%.12f,%.12f]',p(2),p(2),p(2)); 

   k_acc_bias= '[0.0,0.0,0.0]';
  k_acc_bias=   sprintf('[%.12f,%.12f,%.12f]',p(4),p(4),p(4));

  k_ang_bias= '[0.0,0.0,0.0]';
  k_ang_bias=    sprintf('[%.12f,%.12f,%.12f]',p(3),p(3),p(3));  

  ang_bias  = '[0.0,0.0,0.0]';
  ang_bias=   '[-0.000005,0,-0.000015]';

  
  acc_bias=   '[0.0,0.0,0.0]';
  acc_bias=   '[-0.015,0,-0.005]';

  
  LAT=        '39.32';

  % construct input and putput file names
  % KVH inpu file name 
  KVH    =[EXP_dir 'kvh/' LOG_in_fn '.KVH'];
  % Phinps inpu file name 
  PHINS  =[EXP_dir 'phins/' LOG_in_fn '.INS'];
  %  CSV output file name
  CSV    =[EXP_dir 'proc/kvh/processed/' LOG_out_fn '.CSV'];
  %  Config file name
  CONFIG    =[EXP_dir 'proc/kvh/configs/' LOG_out_fn '.m'];
  %  PDF file name
  PDF    =[EXP_dir '/proc/kvh/pdfs/' LOG_out_fn '.pdf'];

 
  % create output directories if they do not exist
  if (7 ~= exist([EXP_dir '/proc']))
    fprintf(1,'tn_opt: Creating %s\n',[EXP_dir '/proc']);
    mkdir(EXP_dir, 'proc');
  end

  if (7 ~= exist([EXP_dir '/proc/kvh']))
    fprintf(1,'tn_opt: Creating %s\n',[EXP_dir '/proc/kvh']);
    mkdir([EXP_dir '/proc'], 'kvh');
  end

  if (7 ~= exist([EXP_dir '/proc/kvh/configs']))
    fprintf(1,'tn_opt: Creating %s\n',[EXP_dir '/proc/kvh/configs']);
    mkdir([EXP_dir '/proc/kvh'], 'configs');
  end  

  if (7 ~= exist([EXP_dir '/proc/kvh/processed']))
    fprintf(1,'tn_opt: Creating %s\n',[EXP_dir '/proc/kvh/processed']);
    mkdir([EXP_dir '/proc/kvh'], 'processed');
  end

  if (7 ~= exist([EXP_dir '/proc/kvh/pdfs']))
    fprintf(1,'tn_opt: Creating %s\n',[EXP_dir '/proc/kvh/pdfs']);     
      mkdir([EXP_dir '/proc/kvh'], 'pdfs');
  end

  % construct and execute command line to generate config file
  config_cmd = ['python ' TrueNorth_Package_dir '/python/gen_config_file.py' ...
                ' -i '           KVH ...
                ' -o '           CSV ...
                ' -c '           CONFIG ...
                ' -z '           HZ ...
                ' --rpy_ro '     rpy_ro ...
                ' --rpy_align '  rpy_align ...
                ' --k_acc '      k_acc ...
                ' --k_E_n '      k_E_n ...
                ' --k_acc_bias ' k_acc_bias ...
                ' --k_ang_bias ' k_ang_bias ...
                ' --acc_bias '   acc_bias ...
                ' --ang_bias '   ang_bias ...
                ' --k_g '        k_g ...
                ' --k_north '    k_north ...
                ' -l '           LAT ...
                ];  

  % echo the command
  fprintf(1,'tn_opt: ---------------------------------------------------------------\n', config_cmd);
  fprintf(1,'tn_opt: executing command: %s\n', config_cmd);
  % execute the command
  [status cmdout] = unix(config_cmd, '-echo');
  % newsy               
  fprintf(1,'tn_opt: cmd status=%d\n',  status);
  fprintf(1,'tn_opt: cmd output= %s\n', cmdout);

  fprintf(1,'tn_opt: ---------------------------------------------------------------\n', config_cmd);

  %   sim_cmd = 'rosrun truenorth post_process ';
  %   sim_cmd = 'printenv | grep ROS';  
  %   postprocess_shell_script = '/home/llw/kvh_catkin_ws/src/truenorth/scripts/tn_run_postprocess.sh';  
  %   sim_cmd = [postprocess_shell_script ' ' CONFIG];
  %

  % 2018-08-29 LLW for some reason rosrun would find the package but
  % would not find the executable, even when called within a shell, so
  % call the executable directly
  sim_cmd = ['/home/spiels/catkin_ws/devel/lib/truenorth/post_process ' CONFIG];
  fprintf(1,'tn_opt: executing command: %s\n', sim_cmd);
  % execute the command
  [status cmdout] = unix(sim_cmd, '-echo');
  % newsy               
  fprintf(1,'tn_opt: cmd status=%d\n',  status);
  fprintf(1,'tn_opt: cmd output= %s\n', cmdout);

  % compute rms error
  err = tn_opt_compute_rms_error_llw(CSV, PHINS, p);
  
  %pause(60);
  
  return;


  
