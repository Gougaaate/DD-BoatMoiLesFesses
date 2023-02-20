import time
import sys
import gps_driver_v2 as gpsdrv
import gpxpy.gpx
import datetime

f = open("aquisitiondedonnéesgps.txt", "w")
gps = gpsdrv.GpsIO()
gpx = gpxpy.gpx.GPX()
# Create first track in our GPX:
gpx_track = gpxpy.gpx.GPXTrack()
gpx.tracks.append(gpx_track)
# Create first segment in our GPX track:
gpx_segment = gpxpy.gpx.GPXTrackSegment()
gpx_track.segments.append(gpx_segment)
tmax = 60  #on essaye pendant 60 secondes
f.write(datetime.today())
to = time.time()
while time.time() - to <= tmax:
    lat, lon = cvt_gll_ddmm_2_dd(gps_data_string)
    f.writelines(lat, lon)
    time.sleep(1.0)
