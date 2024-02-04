from room import ChallengeRoom
from vpython import rate,keysdown

#Create room environment
environment=ChallengeRoom('Eco-Disaster',6)


#Manual controle for debugging
while True:
    rate(50)
    keys=keysdown()
    if 'q' in keys:
        print('quit')
        break
    elif 'a' in keys and 'd' in keys:
        environment.movePiBot('forward')
    elif 'd' in keys:
        environment.movePiBot('left',.4)
#        environment.movePiBot('left',0)
    elif 'a' in keys:
        environment.movePiBot('right',.4)
#        environment.movePiBot('right',0)
    keys=[]
