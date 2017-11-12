package org.firstinspires.ftc.teamcode;

/**
 * Created by eriche on 11/3/17.
 */

public class KrakenOrientation {
    public final byte encoded;
    public final boolean straightToCryptobox;
    public final boolean red;
    public KrakenOrientation (boolean strToCrypto, boolean isRed) {
        this.straightToCryptobox = strToCrypto;
        this.red = isRed;
        if(red) {
            if(strToCrypto) {
                this.encoded=0;
            } else {
                this.encoded=1;
            }
        } else {
            if(strToCrypto) {
                this.encoded=2;
            } else {
                this.encoded=3;
            }
        }
    }
}
