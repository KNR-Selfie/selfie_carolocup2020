#!/usr/bin/env bash
selfie_ip="10.10.1.1"
selfie_name="selfie"

echo "selfie password:"
read -s password

deleted="/home/selfie/selfie_carolocup2020/src"

while getopts ":d" opt; do
  case ${opt} in
    d )
      deleted="${deleted} /home/selfie/selfie_carolocup2020/build /home/selfie/selfie_carolocup2020/devel"
      echo $deleted
      ;;
    \? ) echo "Usage: ./deploy.sh [-d]"
      ;;
  esac
done
echo "start"
sshpass -p "$password" ssh "${selfie_name}@${selfie_ip}" rm -r $deleted && echo "deleted remote workspace"
sshpass -p "$password" scp -p -r src "${selfie_name}@${selfie_ip}":/home/selfie/selfie_carolocup2020/src && echo "workspace copied"

echo "copied"
