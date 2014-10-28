function distance = team106_distanceMetric_linfinitynorm(t1, t2)
    distance = max(diff([t1 t2], 1, 2));
end