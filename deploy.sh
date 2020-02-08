#!/usr/bin/env bash
selfie_ip="10.10.1.1"
selfie_name="selfie"

echo "selfie password:"
read -s password

sshpass -p "$password" ssh "${selfie_name}@${selfie_ip}" rm -r /home/selfie/selfie_carolocup2020/src /home/selfie/selfie_carolocup2020/build /home/selfie/selfie_carolocup2020/devel && echo "deleted remote workspace"
sshpass -p "$password" scp -r src "${selfie_name}@${selfie_ip}":/home/selfie/selfie_carolocup2020/src && echo "workspace copied"

echo "copied"
