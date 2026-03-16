
threeSixty = {
    init: function () {
    this._vr = new AC.VR('viewer', 'images/Frame######.png', [30, 16], {
            invert: false
        });
    },
    didShow: function() {
        this.init();
    },
    willHide: function() {
        recycleObjectValueForKey(this, "_vr");
    },
    shouldCache: function() {
        return false;
    }
}
if (!window.isLoaded) {
    window.addEventListener("load", function() {
        threeSixty.init();
    }, false);
}
