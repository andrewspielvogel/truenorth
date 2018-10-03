function rms_error = tn_opt_compute_rms_error_llw_all(kvh_csv_fn, phins_log_fn, optional_title_string)
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
  
  if (1 ~= exist('optional_title_string'))
    optional_title_string = ' ';
  end

  % load phins data structure if not already loaded
  if(1==isstruct(phins))
    fprintf(1,'tn_opt: Phins data structure exists in Workspace\n');
  else 
    fprintf(1,'tn_opt: Phins data structure does not exist in Workspace\n');
    fprintf(1,'tn_opt: Reading %s\n',phins_log_fn);    
    phins = read_phins_imbat(phins_log_fn);
  end

  % read the kvh csv file from the simulation
  fprintf(1,'tn_opt: Reading KVH data from %s\n', kvh_csv_fn);

  % read the CSV file from the numerical simulation
  % skip first row and, and first column of all records
  % roll, pitch, and heading in radians are in columns 9, 10, and 11, respectively
  kvh = csvread(kvh_csv_fn,1,1);

  % get kvh attitude and times
  kvh_att  = kvh(:,8:10) * (180.0/pi);
  kvh_t    = kvh(:,7);

  % ----------------------------------------------------------------------
  % plot rph in its original RAW wrapped to +/= pi format
  % ----------------------------------------------------------------------  
  figure(1);
  dof_labels = ['ROLL '; 'PITCH'; 'HDG  '];
		 
  for i=1:3
    subplot(3,1,i);
    plot(taxis(kvh_t), kvh_att(:,i), taxis(phins.t), phins.att(:,i));
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('Degree');
    legend('KVH','Phins');
    title(['RAW Attitide: KVH vs Phins ' dof_labels(i,:) ]);
  end
  
  % unwrap kvh heading
  kvh_att(:,3) = unwrap360(kvh_att(:,3));

  % assign phins angles  
  phins_att = phins.att;
  % unwrap phins heading
  phins_att(:,3) = unwrap360(phins.att(:,3));

  % resample kvh attitude to to phins times
  for i = 1:3
    kvh_att_resamp(:,i) = my_resample(kvh_t, kvh_att(:,i), phins.t);
  end 

  % compute rms error, exclude first 30 min
  % nt0 = 30*60*10;
  % compute rms error for entire dataset, do no exclude first 30 min
  nt0 = 1;
  % compute error
  err_att = kvh_att_resamp - phins_att;
  
  % compute rms error for each dof
  rms_error_dof = sqrt(mean(err_att(nt0:end,:).^2))
  
  % compute overall rms error
  rms_error = sqrt(mean(sum((err_att(nt0:end,:).^2)')))

  %  compute weighted RMS error
  % err_att_sq_weighted(:,1) = err_att(:,1).^2 *(0.01);
  %  err_att_sq_weighted(:,2) = err_att(:,2).^2 *(0.01);
  % err_att_sq_weighted(:,3) = err_att(:,3).^2 *(1.0);
  % rms_error = sqrt(mean(sum(err_att_sq_weighted')));
  
  
  % ----------------------------------------------------------------------
  % plot unwrapped rph
  % ----------------------------------------------------------------------  
  pause(0.5);
  figure(2);
  dof_labels = ['ROLL '; 'PITCH'; 'HDG  '];
		 
  for i=1:3
    subplot(3,1,i);
    plot(taxis(kvh_t), kvh_att(:,i), taxis(phins.t), phins_att(:,i));
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('Degree');
    legend('KVH','Phins');
    title(['Unwrapped Attitide: KVH vs Phins ' dof_labels(i,:)]);
  end

  % ----------------------------------------------------------------------
  % plot rph error
  % ----------------------------------------------------------------------
  pause(0.5);
  figure(3);  
  for i=1:3
    subplot(3,1,i);
    plot(taxis(phins.t), err_att(:,i), taxis([phins.t(nt0);phins.t(end)]),[rms_error_dof(i);rms_error_dof(i)] );
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('Degree');
    legend('KVH - Phins',sprintf('RMS=%+.3f deg',rms_error_dof(i)));
    title(['Attitide Error: KVH - Phins ' dof_labels(i,:)]);
  end

  % ----------------------------------------------------------------------
  % plot angular bias
  % ----------------------------------------------------------------------  
  pause(0.5);
  figure(4);  
  for i=11:13
    subplot(3,1,i-10);
    plot(taxis(kvh_t), kvh(:,i));
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('Deg/S');
    title('Anugular Rate Bias');
    legend(sprintf('mean= %+.10f',mean(kvh(:,i))));
  end

  % ----------------------------------------------------------------------
  % plot accel bias
  % ----------------------------------------------------------------------    
  pause(0.5);
  figure(5);  
  for i=17:19
    subplot(3,1,i-16);
    plot(taxis(kvh_t), kvh(:,i));
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('m/s^2');
    title('Acceleration Bias');
    legend(sprintf('mean= %+.10f m/s^2',mean(kvh(:,i))));    
  end
  

  % ----------------------------------------------------------------------
  % plot parameters
  % ----------------------------------------------------------------------
     paramater_names = ['K_g    ',...
  		      'K_north',...
		      'K_acc  ',...
		      'K_E_n  '];  
  
  % cascade
 
  
  
return

