% 2018-09-06 LLW use fmincon to specify lower bound on all gains is zero
% LLW Sim #8 - fmincon start with gain params where #7 concluded, and add 6 bias params initialized to value of converged sim #2, 24 total params

options = optimset('PlotFcns',@optimplotfval);

Parameter_IC = [ 1.822366298178901 0.419521626875132  0.000048047071289 0.781946393261745 0.000498990156877 0.003051693308977 5.153851846200926 12.128401887604852 4.259290326794651 0.000055047696490 0.000008572532618 0.000053442582873 0.117451741785355 1.599267791848306 0.002585545953700 0.000000026497089 0.000042343349402 0.000026435080010 0.000003083163786 0.000001807260271 0.000012498206088 0.008583091534961 -0.001739827097389,-0.006883214529009]

% keep all gains nonnegative, but no constraint on boas terms
Lower_Bound = [-1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 -1 -1 -1 -1 -1 -1]

result = fmincon('tn_opt_llw', Parameter_IC, [], [], [], [], Lower_Bound, [], [], options)




