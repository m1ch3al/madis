git init
git remote add origin https://github.com/m1ch3al/madis.git

git config credential.helper store
git push https://github.com/m1ch3al/madis.git
git config --global credential.helper 'cache --timeout 7200'
