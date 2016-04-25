cat ~/.ssh/id_rsa.pub | ssh {host}@{ip} 'cat>>.ssh/authorized_keys'
