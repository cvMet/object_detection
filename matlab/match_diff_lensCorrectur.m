clear all;
x_axis = 1;
y_axis = 1;
files = dir(strcat('match_logs\', '\*.csv'));
groundtruth_easy = [ones(15,1);zeros(55,1)];
wb_oc = [1 0 1 1 1 1 1 1 1 1 1 0 1 1 1];
rb_oc = wb_oc;
g_oc = [1 1 1 1 1 0 0 0 0 0];
vd_oc = [1 0 1 1 1 1 1 0 0 0 0 0 0 0 0];
m_oc = vd_oc;
all_obj = [wb_oc, rb_oc, g_oc, vd_oc, m_oc];
sorted_matches = [];
for file = 1:length(files)
    filename = strcat(files(file).folder, '\', files(file).name);
    data = load(filename,'-ascii');
    fid = fopen(filename);
    C = [];
    for entry = 1:length(data)
        line1 = fgetl(fid);
        C = [C;strsplit(line1,',')];
        C{entry} = str2double(extractBetween(C{entry},7,length(C{entry})))
        C{entry,2} = str2double(C{entry,2});
    end
    fclose (fid);
    C = cell2mat(C);
    targetNR = C(:,1);
    matchCount = C(:,2);
    [sorted_targetNR, sort_idx] = sort(targetNR, 'ascend');
    sorted_matches = [sorted_matches, matchCount(sort_idx)];
end
match_diff = sorted_matches(:,1)-sorted_matches(:,2);
figure(1)
hold on
stem([1:70], match_diff)
stem(all_obj*0.5)
decr_match_oc = 0;
incr_match_oc = 0;
decr_match_ic = 0;
incr_match_ic = 0;
for oc = 1:length(all_obj)
    if((all_obj(oc)== 1) && match_diff(oc)>0)
       decr_match_oc = decr_match_oc+1; 
    end
    if((all_obj(oc)== 1) && match_diff(oc)<0)
       incr_match_oc = incr_match_oc+1; 
    end
        if((all_obj(oc)== 0) && match_diff(oc)>0)
       decr_match_ic = decr_match_ic+1; 
    end
    if((all_obj(oc)== 0) && match_diff(oc)<0)
       incr_match_ic = incr_match_ic+1; 
    end
end
