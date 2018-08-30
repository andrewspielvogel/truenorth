function err = tn_opt_compute_rms_error(kvh_csv_fn, phins_log_fn)
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

  % get kvh attitude sampled at phins times
  kvh_att  = kvh(:,8:10) * (180.0/pi);
  kvh_t    = kvh(:,7);

  % unwrap kvh heading
  kvh_att(:,3) = unwrap360(kvh_att(:,3));

  % unwrap phins heading
  phins_att(:,3) = unwrap360(phins.att(:,3));

  % resample kvh to phins times
  for i = 1:3
    kvh_att_resamp(:,i) = my_resample(kvh_t, kvh_att(:,i), phins.t);
  end 
 
  % 
  err_att = kvh_att_resamp - phins_att;

  % ----------------------------------------------------------------------
  % plot rph
  % ----------------------------------------------------------------------  
  figure(1);
  dof_labels = ['ROLL '; 'PITCH'; 'HDG  '];
		 
  for i=1:3
    subplot(3,1,i);
    plot(taxis(kvh_t), kvh_att(:,i), taxis(phins.t), phins_att(:,i));
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('Degree');
    legend('KVH','Phins');
    title(['Attitide: KVH vs Phins ' dof_labels(i)]);
  end

  % ----------------------------------------------------------------------
  % plot rph error
  % ----------------------------------------------------------------------
  figure(2);  
  for i=1:3
    subplot(3,1,i);
    plot(taxis(phins.t), err_att(:,i));
    grid on;
    xlabel(tlabel(kvh_t));
    ylabel('Degree');
    legend('KVH - Phins');
    title(['Attitide Error: KVH - Phins ' dof_labels(i)]);
  end

  rms = 
  err = 0;

  
  
return

