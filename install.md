Package armmockup_manager need real-time scheduling priority. Else, this error is generated :

`[ERROR] [1639762670.169167052]: Creating timer thread failed!!`

Increase real-time scheduling priority for current user :

`sudo bash -c 'echo "$SUDO_USER - rtprio 99" > /etc/security/limits.d/robotis-rtprio.conf'`

Or, add current user to robotis group, and increase real-time scheduling priority for this group :

`sudo bash -c 'echo "@robotis - rtprio 99" > /etc/security/limits.d/robotis-rtprio.conf'`

Then, restart system.

[source: https://github.com/ROBOTIS-GIT/ROBOTIS-Framework/issues/62](https://github.com/ROBOTIS-GIT/ROBOTIS-Framework/issues/62)

