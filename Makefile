deploy:
	cargo web deploy --release
	sed -i.bak 's/quincunx.js/\/quincunx\/quincunx.js/g' target/deploy/index.html
	scp target/deploy/* risboo6909@85.217.170.67:nginx/conf/src/quincunx

