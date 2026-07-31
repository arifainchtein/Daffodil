#!/bin/bash
#
# dailyreport.sh telepathon_${YEAR}_${MONTH}_${DAY}"
#
# Usage:
#   sh dailyreport.sh TopTank      -> filters data::jsonb->>'Name' = 'TopTank'
#   sh dailyreport.sh              -> no parameter = no name filter (all telepathons for today)
#
# Builds today's table name as telepathon_<year>_<month>_<day>
# (no leading zeros on month/day, e.g. telepathon_2026_6_22)

set -euo pipefail

NAME="${1:-All}"

# --- Build today's table name ---
# Requires GNU date (default on Linux / Raspberry Pi). %-m / %-d strip leading zeros.
YEAR=$(date +%Y)
MONTH=$(date +%-m)
DAY=$(date +%-d)
TABLE="telepathon_${YEAR}_${MONTH}_${DAY}"

# --- Build WHERE clause ---
if [ "$NAME" = "All" ]; then
    WHERE_CLAUSE=""
else
    # Escape any single quotes in NAME to keep the SQL valid/safe
    SAFE_NAME=$(printf '%s' "$NAME" | sed "s/'/''/g")
    WHERE_CLAUSE="WHERE TRIM(data::jsonb->>'Name') = '${SAFE_NAME}'"
fi

# --- DB connection settings ---
# Override via environment variables, or edit the defaults below.
# Use a ~/.pgpass file (or PGPASSWORD) so you aren't prompted for a password.
PGDATABASE="teleonome"

# --- Build the SQL ---
SQL=$(cat <<EOF
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
   TO_TIMESTAMP((MAX(CASE WHEN dene->>'Name' = 'Purpose' AND deneword->>'Name' = 'Source Original Time' THEN deneword->>'Value' END))::bigint)::time AT TIME ZONE 'Australia/Melbourne' AS source_OT
FROM ${TABLE},
   jsonb_array_elements(data::jsonb->'Denes') AS dene,
   jsonb_array_elements(dene->'DeneWords') AS deneword
${WHERE_CLAUSE}
GROUP BY timeseconds, telepathonname
ORDER BY timeseconds DESC;
EOF
)

# --- Output location ---
# Reports are saved under ./reports by default; override with REPORT_DIR.
REPORT_DIR="${REPORT_DIR:-./reports}"
mkdir -p "$REPORT_DIR"
TIMESTAMP=$(date +%H%M%S)
OUTFILE="${REPORT_DIR}/dailyreport_${NAME}_${YEAR}_${MONTH}_${DAY}_${TIMESTAMP}.csv"

echo "Table:  ${TABLE}"
echo "Filter: ${NAME}"
echo "Run at: $(date)"
echo "---"

# Terminal: human-readable aligned table
psql -d "$PGDATABASE" -c "$SQL"

# File: CSV (with header row)
psql -d "$PGDATABASE" --csv -c "$SQL" > "$OUTFILE"

echo
echo "Saved CSV report to: ${OUTFILE}"