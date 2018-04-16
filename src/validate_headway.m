function validated_headway = validate_headway(headway, lower_limit)
  if headway < lower_limit
    validated_headway = lower_limit;
  else
    validated_headway = headway;
end
