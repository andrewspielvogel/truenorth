function rms_error = tn_opt_compute_rms_error(kvh_csv_fn, phins_log_fn, optional_title_string)
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
    fprintf(1,'tn_opt: Readding %s\n',phins_log_fn);    
    phins = read_phins_imbat(phins_log_fn);
  end

  % read the kvh csv file from the simulation
  fprintf(1,'tnopt: Reading KVH data from %s\n', kvh_csv_fn);

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
    title(['RAW Attitide: KVH vs Phins ' dof_labels(i,:)]);
  end
  
  % unwrap kvh heading
  kvh_att(:,3) = unwrap360(kvh_att(:,3));

  % unwrap phins heading
  phins_att(:,3) = unwrap360(phins.att(:,3));

  % resample kvh attitude to to phins times
  for i = 1:3
    kvh_att_resamp(:,i) = my_resample(kvh_t, kvh_att(:,i), phins.t);
  end 
 
  nt0 = 30*60*10;
  % compute error
  err_att = kvh_att_resamp - phins_att;
  
  % compute rms error for each dof
  rms_error_dof = sqrt(mean(err_att(nt0:end,:).^2))
  
  % compute overall rms error
  rms_error = sqrt(mean(sum((err_att(nt0:end,:).^2)')))
  
    

  % ----------------------------------------------------------------------
  % plot unwrapped rph
  % ----------------------------------------------------------------------  
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
  figure(3);  
  for i=1:3
    subplot(3,1,i);
    plot(taxis(phins.t), err_att(:,i), taxis([phins.t(nt0);phins.t(end)]),[rms_error_dof(i);rms_error_dof(i)] );
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('Degree');
    legend('KVH - Phins',sprintf('RMS=%+.2f deg',rms_error_dof(i)));
    title(['Attitide Error: KVH - Phins ' dof_labels(i,:)]);
  end

  cascade
 
  
  
return

