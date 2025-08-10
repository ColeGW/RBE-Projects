function rmse (data, actual)
r = sqrt( sum((data(:)-actual(:)).^2) / numel(data));