from perception.msg import Block

def left_value(block):
    """left_value returns a value which represents left; a higher value means it is more left
    
    effectively this is the y coordinate in negative direction
    
    Arguments:
        block {perception.msg.Block} -- the block from which the value should be taken
    
    Returns:
        float -- a value which is left sortable
    """
    return -block.pose.pose.position.y

def right_value(block):
    """right_value returns a value which represents right; a higher value means it is more right
    
    effectively this is the opposite of the left value
    
    Arguments:
        block {perception.msg.Block} -- the block from which the value should be taken
    
    Returns:
        float -- a value which is right sortable
    """
    return -left_value(block)

def far_value(block):
    """far_value returns a value which represents far; a higher value means it is more far
    
    effectively this is the x coordinate in negative direction
    
    Arguments:
        block {perception.msg.Block} -- the block from which the value should be taken
    
    Returns:
        float -- a value which is far sortable
    """
    return -block.pose.pose.position.x

def close_value(block):
    """close_value returns a value which represents close; a higher value means it is more close
    
    effectively this is the opposite of the far value
    
    Arguments:
        block {perception.msg.Block} -- the block from which the value should be taken
    
    Returns:
        float -- a value which is close sortable
    """
    return -far_value(block)

def high_value(block):
    """high_value returns a value which represents high; a higher value means it is more high
    
    effectively this is the z coordinate in positive direction
    
    Arguments:
        block {perception.msg.Block} -- the block from which the value should be taken
    
    Returns:
        float -- a value which is high sortable
    """
    return block.pose.pose.position.z

def low_value(block):
    """low_value returns a value which represents low; a higher value means it is more low
    
    effectively this is the opposite of the high value
    
    Arguments:
        block {perception.msg.Block} -- the block from which the value should be taken
    
    Returns:
        float -- a value which is low sortable
    """
    return -high_value(block)


# now comes the python duck typing hack
# we create new functions to the already existing class Block to represent semantics

setattr(Block, "left", left_value)
setattr(Block, "right", right_value)
setattr(Block, "far", far_value)
setattr(Block, "close", close_value)
setattr(Block, "high", high_value)
setattr(Block, "low", low_value)

# compare functions
def left_compare(block1, block2):
    return block1.left() > block2.left()

def right_compare(block1, block2):
    return block1.right() > block2.right()

def far_compare(block1, block2):
    return block1.far() > block2.far()

def close_compare(block1, block2):
    return block1.close() > block2.close()

def high_compare(block1, block2):
    return block1.high() > block2.high()

def low_compare(block1, block2):
    return block1.low() > block2.low()