function distance = team106_distanceMetric_l2norm(t1, t2)
    distance = sqrt(sum(diff([t1 t2], 1, 2).^2));
end