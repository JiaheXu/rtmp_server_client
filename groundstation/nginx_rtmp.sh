#!/bin/bash

NGINX_EXEC="/usr/local/nginx/sbin/nginx"

case "$1" in
    start)
        echo "Starting Nginx RTMP service..."
        sudo $NGINX_EXEC
        sleep 1
        ;;
    stop)
        echo "Stopping Nginx RTMP service..."
        sudo $NGINX_EXEC -s stop
        sleep 1
        ;;
    restart)
        echo "Restarting Nginx RTMP service..."
        sudo $NGINX_EXEC -s stop
        sleep 2
        sudo $NGINX_EXEC
        ;;
    *)
        echo "Usage: $0 {start|stop|restart}"
        exit 1
        ;;
esac

exit 0
