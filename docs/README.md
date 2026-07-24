# Documents

This folder keeps formal project documentation. Temporary HM30 debug notes, link logs, screenshots, and one-off test payloads have been removed from the main workspace.

## Recommended Reading Order

1. `hm30_remote_operation.md`  
   Field checklist for the validated HM30 remote acquisition workflow.

2. `uav_gpr_remote_transmission_final_hm30_plan.md`  
   Final HM30-based data transmission plan and engineering rationale.

3. `air_to_ground_radar_data_transmission_plan.md`  
   Software architecture for air-to-ground radar trace transmission.

4. `remote_ground_station_technical_document.md`  
   Ground station technical design.

5. `remote_ground_station_test_report_with_figures.md` and `remote_ground_station_data_result_report.md`  
   Earlier receiver-side test reports and generated result figures.

6. `HM30 v1.7.pdf` / `HM30 v1.7.txt`  
   Vendor manual and extracted text.

## Runtime Data

Acquisition data is intentionally stored outside this documentation folder:

```text
gpr_runs/      air-side local acquisition data
remote_runs/   ground-side received data
```
