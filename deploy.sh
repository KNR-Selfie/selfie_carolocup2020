#!/usr/bin/env bash
selfie_ip="10.70.13.13"
selfie_name="selfie"

echo "selfie password:"
read -s password

deleted="/home/selfie/selfie_carolocup2020/src"

while getopts ":d" opt; do
  case ${opt} in
    d )
      deleted="${deleted} /home/selfie/selfie_carolocup2020/build /home/selfie/selfie_carolocup2020/devel"
      ;;
    \? ) echo "Usage: ./deploy.sh [-d]"
      ;;
  esac
done
echo "deleting workspace"
sshpass -p "$password" ssh "${selfie_name}@${selfie_ip}" rm -r $deleted && echo "deleted remote workspace"
echo "copying workspace"
sshpass -p "$password" scp -p -r src "${selfie_name}@${selfie_ip}":/home/selfie/selfie_carolocup2020/src && echo "workspace copied"

