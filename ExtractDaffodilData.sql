      SELECT
         TO_TIMESTAMP(timeseconds)::time AT TIME ZONE 'Australia/Melbourne' AS timeseconds,
         telepathonname,
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Battery Current' THEN deneword->>'Value' END) AS batC, 
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Battery Voltage' THEN deneword->>'Value' END) AS batV, 
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Estimated Runtime' THEN deneword->>'Value' END) AS ER,
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Led Brightness' THEN deneword->>'Value' END) AS led,
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Operating Status' THEN deneword->>'Value' END) AS OS,
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Async Data' THEN deneword->>'Value' END) AS AD,
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Light Level' THEN deneword->>'Value' END) AS lux,
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Sleep Time' THEN deneword->>'Value' END) AS sleep,
         MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Wake Time Sec' THEN deneword->>'Value' END) AS wts,
         MAX(CASE WHEN dene->>'Name' = 'Configuration' AND deneword->>'Name' = 'Current Function' THEN deneword->>'Value' END) AS CF,
         TO_TIMESTAMP((MAX(CASE WHEN  dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Source Original Time' THEN deneword->>'Value' END) )::bigint)::time AT TIME ZONE 'Australia/Melbourne' AS source_OT

      FROM telepathon_2026_6_22,
         jsonb_array_elements(data::jsonb->'Denes') AS dene,
         jsonb_array_elements(dene->'DeneWords') AS deneword
      WHERE TRIM(data::jsonb->>'Name') = 'TopTank'
      
      GROUP BY timeseconds, telepathonname
      ORDER BY timeseconds desc;





 SELECT
    TO_TIMESTAMP(timeseconds) AT TIME ZONE 'Australia/Melbourne' AS timestamp_melbourne,
    telepathonname,
     MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Fish Tank Outflow Flow Rate' THEN deneword->>'Value' END) AS fish_flow_rate ,
     TO_TIMESTAMP((MAX(CASE WHEN  dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Source Original Time' THEN deneword->>'Value' END) )::bigint) AT TIME ZONE 'Australia/Melbourne' AS timestamp_melbourne

     FROM telepathon_2026_5_28,
      jsonb_array_elements(data::jsonb->'Denes') AS dene,
     jsonb_array_elements(dene->'DeneWords') AS deneword
WHERE TRIM(data::jsonb->>'Name') = 'Chinampa'
 
GROUP BY timeseconds, telepathonname
ORDER BY timeseconds desc;



\copy (SELECT
    TO_TIMESTAMP(timeseconds) AT TIME ZONE 'Australia/Melbourne' AS timestamp_melbourne,
    telepathonname,
    MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Battery Current' THEN deneword->>'Value' END) AS battery_current, 
    MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Battery Voltage' THEN deneword->>'Value' END) AS battery_voltage, 
    MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Estimated Runtime' THEN deneword->>'Value' END) AS estimated_runtime,
    MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Led Brightness' THEN deneword->>'Value' END) AS led_brightness,
    MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Operating Status' THEN deneword->>'Value' END) AS operating_status,
    MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Async Data' THEN deneword->>'Value' END) AS async_data,
    MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Light Level' THEN deneword->>'Value' END) AS lux,
    MAX(CASE WHEN dene->>'Name' = 'Configuration' AND deneword->>'Name' = 'Current Function' THEN deneword->>'Value' END) AS current_function,

    TO_TIMESTAMP((MAX(CASE WHEN  dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Source Original Time' THEN deneword->>'Value' END) )::bigint) AT TIME ZONE 'Australia/Melbourne' AS timestamp_melbourne

FROM telepathon_2026_6_4,
     jsonb_array_elements(data::jsonb->'Denes') AS dene,
     jsonb_array_elements(dene->'DeneWords') AS deneword
WHERE TRIM(data::jsonb->>'Name') = 'TopTank'
 
GROUP BY timeseconds, telepathonname
ORDER BY timeseconds desc) TO '/home/pi/Teleonome/TopTank.csv' WITH (FORMAT csv, HEADER true);