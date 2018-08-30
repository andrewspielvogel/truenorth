function err = tn_opt_llw(p)
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
  TrueNorth_Package_dir = '/home/llw/kvh_catkin_ws/src/truenorth'

  % exp dir with subdirectories /kvh, /phine, and /proc
  EXP_dir = '/home/llw/llw/sentry_2018/data/2018-sentry-gyro/dives_10hz/sentry494'

  % name of expt file with no suffix such as 2018_08_21_12_45'
  LOG_in_fn  = 'dive_bottom'
  LOG_out_fn = 'dive_bottom_expt01'
  HZ = '10'

  % algorithm parameters NO SPACES BETWEEN NUMBERS OR COMMAS
  rpy_align=  '[-1.5798,0.0573,1.5712]';
  rpy_ro=     '[0.0,0.0,4.01]';

  % gains appearing in the 
  k_g=        '[0.1,0.1,0.1]';
  k_north=    '[0.01,0.01,0.01]';

  k_acc=      '[10.0,10.0,10.0]';
  k_E_n=      '[0.0001,0.0001,0.0001]';

  k_acc_bias= '[0.0,0.0,0.0]';
  k_ang_bias= '[0.0,0.0,0.0]';
  
  acc_bias=   '[0.01,0.0,0.0]';
  acc_bias=   '[0.0,0.0,0.0]';
  #  acc_bias  =  sprintf('[%f,%f,%f]',p(4),p(5),p(6))
  # converged values from LLW fminsearch #2
  acc_bias=   '0.000003083163786,0.000001807260271,0.000012498206088]';


  ang_bias=   '[0.0,0.0,0.0]';
  ang_bias=   '[0.000003,0.0,0.00001]';
  %   ang_bias  =  sprintf('[%f,%f,%f]',p(1),p(2),p(3))
  # converged values from LLW fminsearch #2  
  ang_bias  = '[0.008583091534961,-0.001739827097389,-0.006883214529009]';
  
  LAT=        '32.71';

  % construct input and putput file names
  % KVH inpu file name 
  KVH    =[EXP_dir '/kvh/' LOG_in_fn '.KVH'];
  % Phinps inpu file name 
  PHINS  =[EXP_dir '/phins/' LOG_in_fn '.INS'];
  %  CSV output file name
  CSV    =[EXP_dir '/proc/kvh/processed/' LOG_in_fn '.CSV'];
  %  Config file name
  CONFIG    =[EXP_dir '/proc/kvh/configs/' LOG_out_fn '.m'];
  %  PDF file name
  PDF    =[EXP_dir '/proc/kvh/pdfs/' LOG_out_fn '.pdf'];

 
  % create output directories if they do not exist
  if (7 ~= exist([EXP_dir '/proc']))
    fprintf(1,'tn_opt: Creating %s\n',[EXP_dir '/proc']);;
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
  sim_cmd = ['/home/llw/kvh_catkin_ws/devel/lib/truenorth/post_process ' CONFIG];
  fprintf(1,'tn_opt: executing command: %s\n', sim_cmd);
  % execute the command
  [status cmdout] = unix(sim_cmd, '-echo');
  % newsy               
  fprintf(1,'tn_opt: cmd status=%d\n',  status);
  fprintf(1,'tn_opt: cmd output= %s\n', cmdout);

  % compute rms error
  err = tn_opt_compute_rms_error(CSV, PHINS, [ang_bias acc_bias]);
  
  return;
