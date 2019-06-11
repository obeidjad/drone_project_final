

class MySequencer(Sequencer): 
    def __init__(self):
        super(...)
        self.new_mode('hallway',
                      [[('slide', True), 'detect'], # Phase 1
                       [Wait(['slide', 'detect'], [('detect_door', True), ('Forward', False)]), # Phase 2 : attendre que slide et detect aient fini. Wait(cond, seq)
                        ...,
                        ....)
        self.new_mode('stairs',
                      [[....]])
        ...
                       


"""
new_mode(command_name, seq)

seq : cmd_list
cmd : activ_list # activésen même temps
activ :   (nom, bool)
        | Wait(cond_list, avtiv_list)
cond_list : liste de noms.

"""
                       

def main(args):
    rospy.init_node('Sequencer', anonymous=True)
    sc = Sequencer()
    sc.run()
                       
    #rospy.init_node('send_command', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)


"----------------------"


sequence = { 'hallway' : {....}
               'stairs' : {....}
               }

seq = Sequenceur(sequence) # et ça part....
