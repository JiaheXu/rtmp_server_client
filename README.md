# RTMP Streamer and Server


## On the groundstation or RTMP NGINX serving device:

### Setup
* Dependencies Required for NGINX

```bash
sudo apt install build-essential libpcre3 libpcre3-dev libssl-dev zlib1g zlib1g-dev
```

* Move the nginx folders for install if desired. CD into the nginx-1.27.1 dir, then configure it for rtmp. Modify the EXACT location of the nginx-rtmp directory

```bash
./configure --add-module=/home/starfish_admin/software/nginx-rtmp-module --with-http_ssl_module
```

* Make and install

```bash
make
sudo make install
```

* Modify nginx.conf for IP's and broadcasting for RTMP. See sample nginx.conf in the groundstation folder.
`sudo nano /usr/local/nginx/conf/nginx.conf`

* Modify `nginx-rtmp.service` file with correct users and paths then move to `/etc/systemd/system/`

## On the originating device or vehicle computer:
* Modify `triage-stream.service` file for proper users and paths then move into `/etc/systemd/system`
* Modify `stream.sh` with the IP of the nginx serving device.


## Running
* Register new services and file changes: Run on both devices:
	* `sudo systemctl daemon-reload`
* Starting/restarting service:
	* `sudo systemctl restart ____.service`
* To verify status of operation: `systemctl status ____.service`.
	* Due to startup delay on some service files, it may take a few seconds to pass the "activating" stage.
* Enable service to start on boot:
	* `sudo systemctl enable ____.service`
* Test playback on the groundstation: nobuffer and low_delay are the important flags: `ffplay -fflags nobuffer -flags low_delay -framedrop -strict experimental rtmp://10.223.132.96:1937/live/stream`
