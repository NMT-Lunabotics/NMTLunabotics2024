
void setup() {
    while(true) {
        moveLeft = true;
        moveRight = true;

        leftPos = getLeftPos();
        rightPos = getRightPos();

        if(leftPos - rightPos > maxDiff) {
            moveLeft = false;
        }

        if(moveLeft)
            move(left);
        else if (moveRight)
            move(right)
    }
}
